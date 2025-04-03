# Sliding Window Average

#### Request:

>  Compute the average of the sampled signal over a window, for example, 5 secs.

---

#### Solution:

Using FreeRTOS, two tasks have been created, and a circular buffer approach has been implemented. 

- One task is responsible for reading ADC values and storing them in the `dataQueue`.
- The other task computes the average once the circular buffer is full.

This design ensures that data collection and processing are handled concurrently, while the circular buffer maintains efficient data management.

Note that no **Hampel error measures** or similar methods have been used, as they were not part of the original request.
