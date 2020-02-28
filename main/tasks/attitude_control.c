

void attitude_control() {
     while (1) {
         //update pids
         xErr = imu.pitch;
         xInt = xInt + xErr;
         xDer = xErr - xLast;
         xLast = xErr;
         xPIDout = xPgain * xErr + xIgain * xInt + xDgain * xDer;
         //update Motors
         Motor1 = (int)(throttle - xPIDout - yPIDout - zPIDout );
         Motor2 = (int)(throttle - xPIDout + yPIDout + zPIDout );
         Motor3 = (int)(throttle + xPIDout + yPIDout - zPIDout );
         Motor4 = (int)(throttle + xPIDout - yPIDout + zPIDout );

         if (Motor1 < 1000) Motor1 = 1000; if (Motor1 > 1500) Motor1 = 1500;
         if (Motor2 < 1000) Motor2 = 1000; if (Motor2 > 1500) Motor2 = 1500;
         if (Motor3 < 1000) Motor3 = 1000; if (Motor3 > 1500) Motor3 = 1500;
         if (Motor4 < 1000) Motor4 = 1000; if (Motor4 > 1500) Motor4 = 1500;
         //if (cal == 1) { Motor1 = 2000; Motor2 = 2000; Motor3 = 2000; Motor4 = 2000; }

         mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, Motor1);
         mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, Motor2);
         mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, Motor3);
         mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, Motor4);
 
         vTaskDelay(20/portTICK_RATE_MS); //should run about 50Hz min 
     }
}

