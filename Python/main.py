from time import sleep, time
from turtle import position

import numpy as np

from evaluation import Evaluate_Pick
from harvest import Harvest
from PID_Gains import *
from reset_robot import Reset_Robot
from update import Update_Parametres


def main():
    #------------------------------------------------------------------------------------------------------------------------------------------
    # Initialization values
    #------------------------------------------------------------------------------------------------------------------------------------------
    # Initial desired gripping force
    F_d1 = 350
    # Total number of iterations
    n = 15
    # Number of trials per iteration
    n_trials = 5

    file = open('Training Parametres.txt', 'w+')

    ref_name = "python/referenceMFTPU.csv"

    pull_vec = np.array([0, 0, -0.03]) 

    harv  = Harvest()
    reset = Reset_Robot()
    eval  = Evaluate_Pick()
    upd   = Update_Parametres()

    raspberry = reset.get_arm_pose()

    ref = eval.get_reference(ref_name)
    max_ref = np.max(ref)

    reset.move_arm_to_position(raspberry, False)
    
    # Begin training -----------------------------------------
    iter = 1
    while iter <= n:
        print('iteration '+str(iter))
        iter_trials = []
        i = 0
        retry = False
        while i < n_trials:
            zeroTime = time()
            eval.take_offset()
            output = 2
            while np.abs(output) > 1:
                output = eval.get_data()

            check_stem = True
            time_vec = []
            harvest_data = []
            pulling_force = []
            pulling_gripper= []
            compression_loadcell = []

            #----------------------------------------------------------------------------------------------------------------------------------
            # Grasp and pull
            #----------------------------------------------------------------------------------------------------------------------------------
            Fd = F_d1
            harv.set_controller(K1[0], K1[1], K1[2]) 
            print('set controller')
            harvest_data.append(eval.get_data())
            # harv.grasp_raspberry()
            # sleep(0.5)
            error_margin = 10
            fail = False
            while True:
                error = 12
                while np.abs(error) > error_margin:
                    if harv.is_off_stem_gripper() and check_stem:
                        reset.stop_arm()
                        Fd = F_d2
                        error_margin = 5
                        check_stem = False 
                        print('OFF STEM')
                        harv.set_controller(K2[0], K2[1], K2[2])   
                        counter = 0
                    error, compression_force, _ = harv.control_gripper(Fd, harv.is_off_stem_gripper())
                    t = time() - zeroTime
                    time_vec.append(t)
                    harvest_data.append(eval.get_data())
                    compression_loadcell.append(compression_force)
                    pulling_force.append(eval.get_vertical_force())
                    pulling_gripper.append(harv.get_gripper_vertical_force())
                    t = time() - zeroTime
                    eval.plot_data(compression_force, Fd, eval.get_data(), eval.get_vertical_force(), harv.get_gripper_vertical_force(), t)
                    
                    # If the picking is not detected
                    if abs(reset.get_arm_pose()[2]-raspberry[2]) >= 0.08 and Fd == F_d1:
                        harv.move_gripper(0)
                        reset.stop_arm()
                        fail = True
                        retry = True
                        break
                    elif Fd == F_d2 and harv.is_closed():
                        fail = True
                        break
                if fail:
                    break

                if np.abs(error) <=  error_margin and Fd == F_d1:
                    # Pull raspberry 
                    sleep(0.5)
                    error_margin = 3
                    pos = reset.change_arm(pull_vec, 0)
                    reset.move_arm(pos)

                elif np.abs(error) <=  error_margin and Fd == F_d2:
                    counter += 1
                    print(counter)
                    if counter == 3:
                        break
            harv.move_gripper(0)

            #----------------------------------------------------------------------------------------------------------------------------------
            # Check if raspberry was picked successfully
            #----------------------------------------------------------------------------------------------------------------------------------
            if not retry:
                verify = harv.check_raspberry()

                print('\n\nverified\n\n')

                if (verify[-1]-verify[0]) > 10 and not fail:
                    check_stem = False
                    error = 12
                    while np.abs(error) > error_margin:
                        error, _, _ = harv.control_gripper(F_d2, False)
                    harv.move_gripper(0) 
                    iter_trials.append(max(harvest_data))

                    #--------------------------------------------------------------------------------------------------------------------------
                    # Reset raspberry and robot to position
                    #--------------------------------------------------------------------------------------------------------------------------
                    reset.move_arm_to_position(raspberry, False)
                    harv.open_fingers()
                    harv.reset()
                    eval.reset()

                    #--------------------------------------------------------------------------------------------------------------------------
                    # Register Parametres
                    #--------------------------------------------------------------------------------------------------------------------------
                    file.write('\n\n\n')
                    file.write('Iteration = '+str(iter)+' trial = '+str(i)+'\n')
                    file.write('Fd = '+str(F_d1)+'\n')
                    file.write('Grasp controller: K = '+str(K1)+'\n') 
                    file.write('Compression Loadcell = '+str(compression_loadcell)+'\n')
                    file.write('Time vector = '+str(time_vec)+'\n')
                    file.write('Pulling Force = '+str(pulling_force)+'\n')
                    file.write('Pulling Gripper = '+str(pulling_gripper)+'\n')
                    file.write('Raspberry Reading = '+str(harvest_data)+'\n')            

                else:
                    iter_trials.append(0)
                    #--------------------------------------------------------------------------------------------------------------------------
                    # Register Parametres
                    #--------------------------------------------------------------------------------------------------------------------------
                    file.write('\n\n\n')
                    file.write('Iteration = '+str(iter)+' trial = '+str(i)+'\n')
                    file.write('PICKING FAILED\n')
                    file.write('Fd = '+str(F_d1)+'\n')
                    file.write('Grasp controller: K = '+str(K1)+'\n') 
                    
                    #--------------------------------------------------------------------------------------------------------------------------
                    # Reset raspberry and robot to position
                    #--------------------------------------------------------------------------------------------------------------------------
                    harv.open_fingers()
                    reset.move_arm_to_position(raspberry, False)
                    harv.reset()
                    eval.reset()      
                i += 1
            else:
                retry = False
                #------------------------------------------------------------------------------------------------------------------------------
                # Reset raspberry and robot to position
                #------------------------------------------------------------------------------------------------------------------------------
                harv.open_fingers()
                reset.move_arm_to_position(raspberry, False)
                harv.reset()
                eval.reset()   
        
        #--------------------------------------------------------------------------------------------------------------------------------------
        # Register Parametres
        #--------------------------------------------------------------------------------------------------------------------------------------
        file.write('\n\n\n')
        file.write('Iteration = '+str(iter)+'\n')
        file.write('Max forces before outlier removal'+str(iter_trials)+'\n')
        

        #--------------------------------------------------------------------------------------------------------------------------------------
        # Process and compare data 
        #--------------------------------------------------------------------------------------------------------------------------------------
        # If all trials were successful
        if iter_trials.count(0) == 0:
            max_forces = eval.clip_outliers(np.array(iter_trials), 2)
            dif, relative_error = eval.cost_function(max_ref, np.mean(max_forces))
            file.write('Max forces after outlier removal'+str(max_forces.tolist())+'\n')
        else:
            dif, relative_error = eval.cost_function(max_ref, np.sum(iter_trials)/(n_trials-iter_trials.count(0)+10^(-6)))
        F_d1 = upd.update_Fd(F_d1, dif, iter_trials.count(0), n_trials)

        #----------------------------------------------------------------------------------------------------------------------------------
        # Register Parametres
        #----------------------------------------------------------------------------------------------------------------------------------
        # file.write('Pulling loss = '+str(MSE)+'\n')
        file.write('Pulling force relative error = '+str(relative_error)+'\n')
        file.write('Pulling difference = '+str(dif)+'\n')
        file.write('#############################################################################')
        iter += 1
         
    file.close()


if __name__ == '__main__':
    main()
