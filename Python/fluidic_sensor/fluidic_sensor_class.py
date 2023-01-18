from time import time
import numpy as np
from sklearn.linear_model import LinearRegression

class Fluidic_Sensor: 
    def __init__(self, sensor_offset):

        # Timers
        self.cond1Timer = time()
        self.cond2Timer = time()
        self.ma_sampling_timer = time()

        # Postprocessing tools
        self._define_gaussian(nGaussian=15, sigma=1)
        self._moving_average_tools(100, 0.005)
        self.reg = LinearRegression()
        self.offset_adjustment_gain = 0.01
        self.set_auto_drift_parameters()
        
        # Signals
        self.t = 0
        self.raw = 0
        self.smoothed = 0
        self.ma = 0
        self.dpdt = 0
        self.sensor_pressed = 0

        # Signal offsets
        self.sensor_offset = sensor_offset
        self.offset_adjustment = 0

    # Private methods
    def _define_gaussian(self, nGaussian, sigma):
        x = np.linspace(-nGaussian, nGaussian, nGaussian*2 +1)
        self.nGaussian = nGaussian
        self.gaussian1dKernel = np.exp(-(x/sigma)**2/2) / np.sqrt(2*np.pi) / sigma
        self.smoothed_storage = np.zeros(nGaussian * 2 + 1)

    def _moving_average_tools(self, ma_length, ma_timer_dt):
        self.ma_sample_length = ma_length
        self.ma_storage = np.zeros(ma_length)
        self.ma_time_storage = np.zeros(ma_length)
        self.ma_timer_dt = ma_timer_dt

    def _smoothing(self):
        self.smoothed_storage = np.roll(self.smoothed_storage, 1)
        self.smoothed_storage[0] = self.raw

        smoothed_conv = np.convolve(self.gaussian1dKernel, self.smoothed_storage)
        self.smoothed = smoothed_conv[int(np.floor(len(smoothed_conv)/2))]

    # Public method
    def set_sensor_offset(self, sensor_offset):
        self.sensor_offset = sensor_offset

    def set_auto_drift_parameters( self,cond1_time = 0.1, 
                                        cond1_slope_min = -1, 
                                        cond1_slope_max = 0.1,
                                        cond1_error = 1, 
                                        cond2_time = 3, 
                                        cond2_slope_min = -0.2, 
                                        cond2_slope_max = 0.1,
                                        cond2_error = 2):

        self.cond1_time_threshold = cond1_time
        self.cond2_time_threshold = cond2_time

        self.cond1_slope_min = cond1_slope_min
        self.cond1_slope_max = cond1_slope_max
        self.cond2_slope_min = cond2_slope_min
        self.cond2_slope_max = cond2_slope_max

        self.cond1_error = cond1_error
        self.cond2_error = cond2_error
  
    def smooth_signal(self, sensor_reading, t):
        self.raw = sensor_reading
        self.t = t

        # Perform smoothing operation here
        self.smoothed_storage = np.roll(self.smoothed_storage, 1)
        self.smoothed_storage[0] = self.raw

        smoothed_conv = np.convolve(self.gaussian1dKernel, self.smoothed_storage)
        self.smoothed = smoothed_conv[int(np.floor(len(smoothed_conv)/2))]

    def get_processed_signal(self):
        return self.smoothed - self.sensor_offset, self.smoothed - self.sensor_offset - self.offset_adjustment

    def auto_drift_compensation(self, sensor_reading, t):

        # Perform smoothing
        self.smooth_signal(sensor_reading, t)

        # Collect data in order to get ma of signal
        # MA is only used to find the dp/dt data, needed to do automatic drift compensation
        if time() - self.ma_sampling_timer > self.ma_timer_dt:

            self.ma_storage = np.roll(self.ma_storage, 1)
            self.ma_time_storage = np.roll(self.ma_time_storage, 1)

            self.ma_storage[0] = self.smoothed
            self.ma_time_storage[0] = self.t

            self.ma_sampling_timer = time()

        # Compute ma
        self.ma = np.average(self.ma_storage)

        # Find current error (difference between initial sensor reading and current)
        error = self.smoothed - self.sensor_offset - self.offset_adjustment

        # Slope and fit
        self.reg.fit(self.ma_time_storage.reshape(-1,1), self.ma_storage)
        self.dpdt = self.reg.coef_[0]

        if self.t > 1:
            # Condition 1
            if self.dpdt > self.cond1_slope_max or self.dpdt < self.cond1_slope_min or error > self.cond1_error:
                self.cond1Timer = time()
            
            # Condition 2
            if self.dpdt > self.cond2_slope_max or self.dpdt < self.cond2_slope_min or error > self.cond2_error:
                self.cond2Timer = time()

            if time() - self.cond1Timer > self.cond1_time_threshold or\
                        time() - self.cond2Timer > self.cond2_time_threshold:

                self.sensor_pressed = 0
                self.offset_adjustment += error * self.offset_adjustment_gain

            else:
                self.sensor_pressed = 1


        