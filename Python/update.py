import numpy as np



class Update_Parametres():
    """
    This class updates Force setpoint 1 after each iteration.
    """
    def __init__(self) -> None:
        self.alpha                  = 10 # update parametre for Fd
        self.Fd_max                 = 400
        self.Fd_min                 = 100
        self.beta_pg                = 0.0001   # update parametre for Kp grasp
        self.beta_pp                = 0.001    # update parametre for Kp pull
        self.beta_i                 = 0.00000 # update parametre for Ki
        self.previuos_successful_Fd = [400]

    def init_Fd(self, reference, Fd_0):
        """
        Initialise Force setpoint 1.
        """
        return np.ones_like(reference)*Fd_0

    def update_Fd(self, Fd, dif, n_fails, n_trials):
        """
        Calculate Force setpoint 1 for next iteration.
        """
        print('updating force')
        Fd_max = self.Fd_max
        if n_fails != 0:
            i = -1
            while self.previuos_successful_Fd[i] == Fd or self.previuos_successful_Fd[i] < Fd:
                i += -1
            Fd1 = Fd + np.abs(Fd-self.previuos_successful_Fd[i])* n_fails/n_trials
            print('update 1\n')
            print(np.abs(Fd-self.previuos_successful_Fd[i])* n_fails/n_trials)
            self.previuos_Fd = Fd
            if Fd1 > Fd_max:
                Fd1 = Fd_max
            return Fd1
        else:
            self.previuos_successful_Fd.append(Fd)
            self.previuos_Fd = Fd
            if dif > 0:
                delta_F = np.random.randint(1,11)
                Fd = Fd - delta_F 
                print('update 2\n')
                print(-delta_F )
                if Fd < self.Fd_min:
                    Fd = self.Fd_min
                return Fd
            else:
                print('update 3\n')
                print(self.alpha*dif)
                Fd = Fd + self.alpha*dif
                if Fd > Fd_max:
                    Fd = Fd_max
                if Fd < self.Fd_min:
                    Fd = self.Fd_min
                return Fd



    





