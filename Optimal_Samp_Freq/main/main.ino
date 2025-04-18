/*
 * Internet of Things - Algorithms and Services  
 *
 * Simple adaptive sampling program. 
 * The program is divided into different tasks: 
 * - one for oversampling,
 * - one for calculating the FFT and possibly adapting the sampling frequency,
 * - one for displaying the available resources.
 * 
 * The program uses a semaphore and task notifications to synchronize 
 * the sampling task and the FFT analysis task. Task notifications 
 * ensure that the FFT is not computed before a complete set of samples 
 * has been collected. The semaphore ensures that the sampling task 
 * does not modify the samples array while it is being referenced by 
 * FFT.compute(), and that the adaptation task does not alter the sampling 
 * frequency during the collection of a set of samples.
 *
 * An attempt at detecting signal changes to determine when to recompute
 * the FFT and adjust the sampling frequency is made using the following logic:
 * if the sampled signal changes significantly at the current sampling rate, 
 * there should be a noticeable variation in the signal's average. To detect 
 * whether the signal has changed too much, the standard deviation is used.
 * However, this approach is not ideal because it only considers the last set 
 * of samples, meaning that gradual signal changes may not trigger FFT 
 * recomputation. Another drawback is the computational overhead of calculating 
 * the mean and standard deviation, which must be done immediately after sampling. 
 * This introduces delays and discrepancies between successive sets.
 * 
 * Therefore, this method is not recommended in most cases. Alternative 
 * approaches could involve interrupts, network commands, other statistical 
 * methods, or timers.
 * In this code timers have been used, triggering the FFT recomputing once
 * every 5 seconds.
 */
#include <Arduino.h>                    
#include "arduinoFFT.h"                 // Slow library, other libraries like CMSIS-DSP or ESP-DSP could be faster due to specific ARM optimizations (basically optimized for devices like the ESP32)
#include "esp_dsp.h"                    // ESP32 optimized FFT library

// For a connection via I2C to the display using the Arduino Wire include:
#include <Wire.h>               
#include "HT_SSD1306Wire.h"

#define ERROR_MARGIN      3             // How far from the standard deviation should the new signal average be, to trigger an FFT recomputation? (Currently 3 times the std. dev.)
#define MONITOR_RESOURCES true          // Should the monitoring task be created? 
#define OLED_ENABLE       true          // Should the performance by displayed on the integrated OLED display?
#define FFT_TIMER         5000          // If using TimerTriggerTask, how much should the task wait before recomputing the FFT?
#define MONITORING_TIMER  2000          // If using MonitoringTask, how often should the task be run to monitor the resources?

const int adcPin = 7;                   // Physical ADC pin used for sampling 
const uint16_t samples = 128;           // Must be a power of 2 - frequency and accuracy dependant - choose based on desired goal

// Initial Sampling Frequency (16 kHz) - oversampling at start
double samplingFrequency = 16000.0f;
double recomputedSamplingFrequency = samplingFrequency;

// FFT active and support buffers
double * vReal;
double * vImag;
double * vRealCalc;
double * vImagCalc;

// FFT Object
float fft_input[samples * 2];           // ESP_DSP

// Task Handles - reference to the task functions, helpful for example to notify the FFT task from the sampling task
TaskHandle_t MonitorTaskHandle = NULL;
TaskHandle_t SamplingTaskHandle = NULL;
TaskHandle_t ProcessingTaskHandle = NULL;
TaskHandle_t OutliersCheckTaskHandle = NULL;
TaskHandle_t TimerTriggerTaskHandle = NULL;

// FFT flags
bool bRecomputeFFT = true;              // Flag telling the sampling task if it should also trigger an FFT recalculation, keep true to trigger the initial adaptation
bool bStatisticalTrigger = false;       // Should mean and standard deviation be used for deciding when to trigger the FFT recalculation? Change as you prefer
bool bTimerTrigger = true;              // Should a timer be used to trigger the FFT recomputation? Change as you prefer

// Buffer handling
enum BufferState { BUFFER_EMPTY, BUFFER_READY, BUFFER_PROCESSING };
volatile BufferState bufferState = BUFFER_EMPTY;

// Statical variables to trigger the FFT recompute 
float stdDeviation = 0.0f;
float mean = 0.0f; 

// Semaphore for critical section handling - it's recommended to initialize it as a binary semaphore
SemaphoreHandle_t xSemaphore;

// Display
static SSD1306Wire  display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

/*
 * Calculate the accurate ADC value - from Heltec example codes
 * Added an improved polynomial, use either, comment out as required
 */
double ReadVoltage(byte pin)
{
    // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
    double reading = analogRead(pin);
    if(reading < 1 || reading >= 4095)
    {
        //return 0;
    }
    // return -0.000000000009824 * pow(reading,3) + 0.000000016557283 * pow(reading,2) + 0.000854596860691 * reading + 0.065440348345433;
    return -0.000000000000016 * pow(reading,4) + 0.000000000118171 * pow(reading,3)- 0.000000301211691 * pow(reading,2)+ 0.001109019271794 * reading + 0.034143524634089;
}

/*
 * Calculate the peak frequency using the arduinoFFT library
 * Execution time: ~1.60ms with 64 samples
 * Execution time: ~27.50ms with 1024 samples
 * Less initial overhead due to accessing directly vReal and vImag, slower calculations - O(nlogn) complexity according to google
 * The FFT object is being reinitialized every time due to continuous buffer swaps. This overhead should be acceptable, given that
 * the FFT will be recomputed rarely compared to other tasks.
 */
double ComputePeakFrequency_Arduino()
{
    // Reinit the ArduinoFFT due to buffer swaps - non optimal operation
    ArduinoFFT<double> FFT(vRealCalc, vImagCalc, samples, samplingFrequency);

    // Compute FFT - needed preprocessing -> get sliding window -> go to frequency domain -> numbers to signal power of each frequency f
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();

    // Return the peak frequency
    return FFT.majorPeak();
}

/*
 * Calculate the peak frequency using the ESP-DSP library - optimized using ARM hardware features and assembly code benath the called functions!
 * API: https://docs.espressif.com/projects/esp-dsp/en/latest/esp32/esp-dsp-apis.html#fft
 * Examples: https://github.com/espressif/esp-dsp/tree/master/examples
 * Execution time: ~3.80ms with 64 samples
 * Execution time: ~6.70ms with 1024 samples
 * More initial overhead due to copying vReal and vImag in a different array, faster computations due to ARM hardware features and binary optimization
 * Overhead could be avoided by writing directly into fft_input when sampling
 * IMPORTANT: Accuracy and resolution highly depends on the number of samples
 */
double ComputePeakFrequency_ESP()
{
    // Initialize the ESP-DSP library
    dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);

    // Prepare input data: interleave real input with zeros for imaginary part as required by the esp fft library
    for (int i = 0; i < samples; i++) 
    {
        fft_input[2 * i] = vRealCalc[i];        // Real part
        fft_input[2 * i + 1] = 0.0f;            // Imaginary part
    }

    // FFT.windowing() - Apply window function to the input data
    dsps_wind_hann_f32(fft_input, samples);

    // FFT.compute() - Perform the FFT - bit reversal method as shown in the examples
    dsps_fft2r_fc32(fft_input, samples);
    dsps_bit_rev_fc32(fft_input, samples);

    // Convert complex result to real/imaginary pair format -> useful for displaying the calculated FFT and also for iterating over the results below
    dsps_cplx2reC_fc32(fft_input, samples);

    // FFT.complexToMagnitude() - Scan from high to low frequencies
    int highestIndex = -1;
    float maxMagnitude = -1.0f;
    for (int i = 1; i < samples / 2; i++)
    {
        float real = fft_input[2 * i];
        float imag = fft_input[2 * i + 1];
        float magnitude = sqrtf(real * real + imag * imag);

        // Stop at the first frequency with a magnitude higher than zero - set threshold to 1e-6f, might need some tweaking
        if (magnitude > maxMagnitude && magnitude > 1e-3f)
        {
            maxMagnitude = magnitude;
            highestIndex = i;
        }
    }

    // If no frequency found - return 0
    if (highestIndex == -1) { return 0.0; }

    // Convert fft_input index to the actual frequency
    double peakFrequency = ((double)highestIndex) * samplingFrequency / samples;
    return peakFrequency;
}

/*
 * Initialize the serial communication, setups the ADC pin for sampling, initialize the needed semaphore and the tasks, running only once at device startup
 */
void setup() 
{
    // Init Serial Console
    Serial.begin(115200);
    Serial.println("Starting Adaptive Sampling with FreeRTOS...");

    // Init ADC
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // Init led
    pinMode(LED ,OUTPUT);

    // Init display
    if (OLED_ENABLE)
    {
        pinMode(Vext,OUTPUT);
        digitalWrite(Vext, LOW);
        display.init();
        display.setFont(ArialMT_Plain_10);
    }

    // Allocate the necessary memory for each buffer
    vReal     = (double*)malloc(samples * sizeof(double));
    vImag     = (double*)malloc(samples * sizeof(double));
    vRealCalc = (double*)malloc(samples * sizeof(double));
    vImagCalc = (double*)malloc(samples * sizeof(double));

    // Init semaphore
    xSemaphore = xSemaphoreCreateBinary();                                                                                      // No end of program - never destroy semaphore
    xSemaphoreGive(xSemaphore);                                                                                                 // Give the semaphore initially so one task can take it

    // Init tasks - sampling task
    xTaskCreatePinnedToCore(SamplingTask, "SamplingTask", 4096, NULL, 1, &SamplingTaskHandle, 0);                               // Run on core 0, highest priority of 1

    // Outlier detection task
    if(bStatisticalTrigger)
    {
        xTaskCreatePinnedToCore(OutliersCheckTask, "OutliersCheckTask", 2048, NULL, 2, &OutliersCheckTaskHandle, 1);            // Run on core 1
    }

    // FFT timer task
    if(bTimerTrigger)
    {
        xTaskCreatePinnedToCore(TimerTriggerTask, "TimerTriggerTask", 4096, NULL, 2, &TimerTriggerTaskHandle, 1);               // Run on core 1
    }

    // Resource displaying task
    if(MONITOR_RESOURCES)
    {
        xTaskCreatePinnedToCore(MonitorTask, "MonitorTask", 2048, NULL, 3, &MonitorTaskHandle, 1);                              // Run on core 1
    }
}

/*
 * Function to handle the buffer swapping
 * Semaphore is important to avoid buffer swapping handled improperly while reading from the ADC, possibly resulting in partially updated buffers
 * Also bufferState tells if the buffers have been already swapped, this is useful for other tasks. This way if more tasks call this function, the
 * buffers will be swapped just once after a frame reading, ensuring that the sampling task will be able to write on the older values, while
 * the computing tasks will be able to read the newest values despite possibly calling many times this function without having newer values.
 */
bool SwapBuffers()
{
    // Never swap buffers if there isn't any newer full frame or if the ADC is sampling the new frame
    if( bufferState == BUFFER_READY )
    {
        if( xSemaphore == NULL )
        {
            Serial.println("[Semaphore] Semaphore error in Processing Task");
        }
        else
        {
            float frameTime = (1000000 / samplingFrequency) * samples * 2;
            if( xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(frameTime) ) == pdTRUE ) // Take semaphore - enter critical section to avoid sampling frequency changes during one frame readings
            {
                // Change this value only when buffers will be swapped for sure
                bufferState = BUFFER_PROCESSING;

                // Temporary pointers 
                double * vReal_tmp;
                double * vImag_tmp;

                // Swap the buffers
                vReal_tmp = vReal;
                vImag_tmp = vImag;
                vReal = vRealCalc;
                vImag = vImagCalc;
                vRealCalc = vReal_tmp;
                vImagCalc = vImag_tmp;

                // Free semaphore
                xSemaphoreGive(xSemaphore); 

                // Has the swapping been successfull?
                return true;
            }
            else
            {
                Serial.println("[Semaphore] Could not take semaphore in Swapping Task");
            }
        }   
    }

    return false;
}

/*
 * A simple sampling task that runs continuously, running concurrently during the FFT computation and the sampling rate adaptation
 * The main issue is avoiding the read and write on the same array of values (the input signal), accessing and modifying vReal and vImag should be done in a critical section
 * as explained in the SwapBuffers function comment, so that partial buffer writes will be avoided. 
 * Another important aspect of this function is the sampling rate adaptation based on the Computing Task computed frequency. The reason to do so is explained below
 * Other functionalities of this function are calculating the time needed to sample, and mayve instantiating a task that detects if the FFT must be recomputed
 */
void SamplingTask(void *parameter) 
{
    while (1) 
    {
        // Comment this part to avoid extra delays when performance evaluation is not needed!
        // uint32_t startTime = micros();

        // Update the sampling frequency if needed - working with doubles, the values might not be the same bit by bit, so check how distant they are
        if( abs(recomputedSamplingFrequency - samplingFrequency) > 1e-1)
        {
            samplingFrequency = recomputedSamplingFrequency;
        }

        if( xSemaphore == NULL )
        {
            Serial.println("[Semaphore] Semaphore error in Sampling Task");
        }
        else
        {
            // Take semaphore - enter critical section to avoid sampling frequency changes during one frame readings
            if( xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(200) ) == pdTRUE )     // Lower sampling frequencies -> higher time to sample and to free semaphore -> higher probability of Swapping tasks not entering the critical sections with small timers
            {
                // Sample the ADC for a frame
                for (int i = 0; i < samples; i++)
                {
                    // vReal[i] = analogRead(adcPin);   // Fast reading
                    vReal[i] = ReadVoltage(adcPin);     // Accurate reading 
                    vImag[i] = 0.0;                     // No imaginary component

                    // Delay based on adaptive frequency - for small frequencies it becomes too big and might be interpreted as getting stuck by FreeRTOS ( Triggering watchdogs ), so use vTaskDelays
                    if( samplingFrequency < 1000.0 )
                    {
                        vTaskDelay(pdMS_TO_TICKS(1000 / samplingFrequency));
                    }
                    else
                    {
                        delayMicroseconds(1000000 / samplingFrequency);
                    }
                }

                // Allow buffer swapping after sampling a full frame
                bufferState = BUFFER_READY;

                // Free the semaphore
                xSemaphoreGive(xSemaphore);
            }
            else
            {
                Serial.println("[Semaphore] Could not take semaphore in Sampling Task");
            }
        }

        // Comment also this part to avoid performance evaluation delays
        // uint32_t elapsedTime = micros() - startTime;
        // Serial.print("[Sampling] Execution Time: ");
        // Serial.print(elapsedTime / 1000.0);
        // Serial.println(" ms");

        // Tell timer trigger that data is available
        if(TimerTriggerTaskHandle)
        {
            xTaskNotifyGive(TimerTriggerTaskHandle);
        }

        // Check if the FFT must be recomputed, and if needed recompute it
        if(bStatisticalTrigger)
        {
            // Swap the buffers only when triggering some sort of calculation over it, like sliding window averages or FFT recomputation
            if( SwapBuffers() )
            {
                Serial.println("[Sampling] Checking for outliers...");
                xTaskNotifyGive(OutliersCheckTaskHandle);
            }
        }

        // Prevent watchdog reset, but again, we want to avoid useless delays, other solutions?
        vTaskDelay(pdMS_TO_TICKS(1));  // Unfortunately, resetting the wathcdog forcefully will cause issues
    }
}

/*
 * Function to check if the FFT must be computed again
 * The idea behind it is that if a signal is consistent and doesn't change much over time, it will have similar
 * mean and standard deviation at each frame. It doesn't work effectively though. Don't rely on this too much.
 */
void OutliersCheckTask(void *parameter)
{
    while(1)
    {
        // Wait until data is available
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Naive statistical approach to detect signal changes
        long sum = 0;                        // Used to compute the average
        for (int i = 0; i < samples; i++)
        {
            sum += vRealCalc[i];
        }

        float newMean = sum / samples;  // average of signal in order to calculate standard deviation
        float newStdDeviation = 0;      // Calculate the standard deviation
        for(int i = 0; i < samples; i++)
        {
            newStdDeviation += pow(vRealCalc[i] - newMean, 2);          // Power of 2
        }
        newStdDeviation = sqrt(newStdDeviation / samples);

        if(newMean >= mean + stdDeviation * ERROR_MARGIN || newMean <= mean - stdDeviation * ERROR_MARGIN )
        {
            bRecomputeFFT = true;
        }

        mean = newMean;
        stdDeviation = newStdDeviation;
        Serial.print("[Adapting] Signal mean: ");
        Serial.println(mean);
        Serial.print("[Adapting] Signal standard deviation: ");
        Serial.println(stdDeviation);

        // Notify FFT task that data is ready
        if(bRecomputeFFT)
        {
            bRecomputeFFT = false;
            Serial.println("[FFT recomputation] FFT recompute triggered due to possible signal changes...");
            xTaskCreatePinnedToCore(ProcessingTask, "ProcessingTask", 2048, NULL, 2, &ProcessingTaskHandle, 1);         // Run on core 1, lower priority of 2
        }
    }
}

/*
 * Task that computes the FFT and then tells the adapted sampling frequency
 * Notice that it's not blocking to the sampling frequency, as garbage samples are avoided by using different buffers for writing and reading
 * Why instantiate and destroy the task each time? Because theoretically the FFT will be recomputed once in a while
 */
void ProcessingTask(void *parameter) 
{
    uint32_t startTime = micros();

    // Get max frequency
    double peakFrequency = ComputePeakFrequency_Arduino();
    Serial.print("[Processing] Peak Frequency: ");
    Serial.println(peakFrequency, 2);
    if (peakFrequency > 0) 
    {
        // Nyquist theorem
        double newSamplingFreq = 2 * peakFrequency;

        // Constrain within limits (10 hz, max frequency found in other code)
        recomputedSamplingFrequency = constrain(newSamplingFreq, 10.0, 33000.0); // Let the other task (the sampling task) decide what to do with the new value (in this code change the sampling frequency before sampling a new frame)
    }
    else
    {
        Serial.println("[Processing] Could not compute peak frequency in Proessing Task");
    }

    uint32_t elapsedTime = micros() - startTime;
    Serial.print("[Processing] Execution Time: ");
    Serial.print(elapsedTime / 1000.0);
    Serial.println(" ms");

    bufferState = BUFFER_EMPTY;

    // Destroy this task
    vTaskDelete(NULL);
}

/*
 * Simply display some informations related to performance and resources on the serial consoles
 * Run once every 2 seconds, display the stack mark (the higher the better), and the available heap space
 * Why this metrics? We are running a resource intensive task (the FFT computing), on a limited memory and computing power device (ESP32) 
 */
void MonitorTask(void *parameter) 
{
    while (1) 
    {
        // Led start blink
	    digitalWrite(LED, HIGH); 

        UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        uint32_t freeHeap = esp_get_free_heap_size();

        // Serial console print
        Serial.println("------ Performance Metrics ------");
        Serial.print("Free Heap Memory: ");
        Serial.print(freeHeap);
        Serial.println(" bytes");

        Serial.print("Sampling Frequency: ");
        Serial.print(samplingFrequency);
        Serial.println(" Hz");

        Serial.print("Task Stack High Water Mark: ");
        Serial.println(uxHighWaterMark);

        // Display print
        if (OLED_ENABLE)
        {
            display.clear();
            display.setTextAlignment(TEXT_ALIGN_LEFT);
            display.drawString(0, 3, "===Performance Metrics===");
            display.drawString(0, 15, "Free Heap:");
            display.drawString(64, 15, String(freeHeap));
            display.drawString(0, 27, "Samp Freq:");
            display.drawString(64, 27, String(samplingFrequency));
            display.drawString(0, 39, "Stack Mark:");
            display.drawString(64, 39, String(uxHighWaterMark));
            display.drawString(0, 51, "=========================");
            display.display();
        }

        // Led end blink
        digitalWrite(LED, LOW);

        vTaskDelay(pdMS_TO_TICKS(MONITORING_TIMER));  // Monitor every 2s 
    }
}

/*
 * Just recompute the FFT once in a while by instantiating it's task
 * Currently done once every 5s + the SwapBuffers() delay (caused by the use of semaphores)
 */
void TimerTriggerTask(void *parameter)
{
    while (1) 
    {
        // Wait until data is available - no need to recompute the FFT if the sampler isn't sampling, in that case stop this task indefinitely until some samples aren't produced
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        Serial.println("[Timer] Periodic FFT recomputation triggered");

        if( SwapBuffers() )
        {
            Serial.println("[Timer] Swapped buffers, recompute FFT");
            xTaskCreatePinnedToCore(ProcessingTask, "ProcessingTask", 2048, NULL, 2, &ProcessingTaskHandle, 1);         // Run on core 1, lower priority of 2
        }

        vTaskDelay(pdMS_TO_TICKS(FFT_TIMER));
    }
}

/*
 * Loop function, useless due to FreeRTOS using and managing of the tasks
 */
void loop() 
{
    // Nothing here again
    vTaskDelete(NULL);      // Just remove the loop function from the FreeRTOS schedule
}
