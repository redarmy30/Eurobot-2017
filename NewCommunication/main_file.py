import  cmd_list
import  driver
import  packets
import time

def stm_test():
    d = driver.Driver(1, 2, 3)
    d.connect()
    #command = {'source': 'fsm', 'cmd': 'echo', 'params': 'ECHO'}
    #print d.process_cmd(command)
    #command = {'source': 'fsm', 'cmd': 'switchOnPneumo','params': ''}
    #print d.process_cmd(command)
    command = {'source': 'fsm', 'cmd': 'setCoordinates', 'params': [0.0,0.0,0.0]}
    print d.process_cmd(command)
    command = {'source': 'fsm', 'cmd': 'addPointToStack', 'params': [0.8,0.0,0.0,4]}
    print d.process_cmd(command)
    #time.sleep(5)
    #command = {'source': 'fsm', 'cmd': 'switchOffPneumo','params': ''}
    #print d.process_cmd(command)


stm_test()

