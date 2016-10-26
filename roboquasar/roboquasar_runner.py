import time
from roboquasar_bot import RoboQuasarBot

class RoboQuasarRunner(RoboQuasarBot):
    def __init__(self, log_data=True):
        super(RoboQuasarRunner, self).__init__("track field checkpoints.gpx", log_data=log_data)
        self.checkpoint_num = 0
        
        self.start()
    
    def button_dn(self, button, params):
        if button == 'A':
            self.record('checkpoint', checkpoint_num=self.checkpoint_num)
            print("Checkpoint %i recorded!" % self.checkpoint_num)
            
            self.checkpoint_num += 1
    
    def main(self):
        time.sleep(0.1)

RoboQuasarRunner(log_data=True).run()
