

void attitude_control() {
     while (1) {
         if (kill == 0 && wait == 0){
            //update pids
            xErr = imu.pitch;
            xInt = xInt + xErr;
            xDer = xErr - xLast;
            xLast = xErr;
            xPIDout = xPgain * xErr + xIgain * xInt + xDgain * xDer;

            yErr = imu.roll;
            yInt = yInt + yErr;
            yDer = yErr - yLast;
            yLast = yErr;
            yPIDout = yPgain * yErr + yIgain * yInt + yDgain * yDer;

            //update Motors
            Motor1 = (int)(throttle - xPIDout - yPIDout - zPIDout );
            Motor2 = (int)(throttle - xPIDout + yPIDout + zPIDout );
            Motor3 = (int)(throttle + xPIDout + yPIDout - zPIDout );
            Motor4 = (int)(throttle + xPIDout - yPIDout + zPIDout );
         }
         else {
            xInt = 0; yInt = 0; zInt = 0;
            xPIDout = 0; yPIDout = 0;
            Motor1 = 1000; Motor2 = 1000; Motor3 = 1000; Motor4 = 0;
         }



         int propmax = 1300;
         if (Motor1 < 1000) Motor1 = 1000; if (Motor1 > propmax) Motor1 = propmax;
         if (Motor2 < 1000) Motor2 = 1000; if (Motor2 > propmax) Motor2 = propmax;
         if (Motor3 < 1000) Motor3 = 1000; if (Motor3 > propmax) Motor3 = propmax;
         if (Motor4 < 1000) Motor4 = 1000; if (Motor4 > propmax) Motor4 = propmax;
         if (cal == 1) { Motor1 = 2000; Motor2 = 2000; Motor3 = 2000; Motor4 = 2000; }

         mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, Motor1);
         mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, Motor2);
         mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, Motor3);
         mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, Motor4);
 
         vTaskDelay(20/portTICK_RATE_MS); //should run about 50Hz min 
     }
}

