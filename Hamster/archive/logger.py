# logger.py
import os

class RunLogger:
    """
    Optional CSV logger.
    Only used if ENABLE_LOGGING is True.
    """
    def __init__(self, folder):
        self.folder = folder
        self._ensure_folder()

    def _ensure_folder(self):
        try:
            os.mkdir(self.folder)
        except OSError:
            pass

    def next_run_number(self):
        try:
            files = os.listdir(self.folder)
        except OSError:
            files = []

        max_run = 0
        for f in files:
            if f.startswith("run_") and f.endswith(".csv"):
                try:
                    num = int(f[4:7])
                    if num > max_run:
                        max_run = num
                except ValueError:
                    pass
        return max_run + 1

    def save_csv(self, run_number, target_rpm, data_rows):
        """
        data_rows: list of tuples
          (time_s, target_rpm, left_rpm, right_rpm, left_pwm, right_pwm, err_l, err_r)
        """
        filename = "{}/run_{:03d}_{:.0f}rpm.csv".format(self.folder, run_number, target_rpm)
        with open(filename, "w") as f:
            f.write("time_s,target_rpm,left_rpm,right_rpm,left_pwm,right_pwm,err_l,err_r\n")
            for row in data_rows:
                f.write("{:.2f},{:.0f},{:.1f},{:.1f},{:.2f},{:.2f},{:.2f},{:.2f}\n".format(*row))
        return filename