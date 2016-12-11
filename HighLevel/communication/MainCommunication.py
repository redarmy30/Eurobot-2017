import stmDriver
import multiprocessing

reply_to_fsm_queue = multiprocessing.Queue()
input_command_queue = multiprocessing.Queue()


# Function add stm_task to queue
def stm_driver(command, parameters = ''):
    command = {'request_source': 'fsm', 'command': command, 'parameters': parameters}
    input_command_queue.put(command)
    return reply_to_fsm_queue.get()

# stm started
stm = multiprocessing.Process(target=stmDriver.stm_loop, args=(input_command_queue,reply_to_fsm_queue))
stm.start()


# use stm_driver to send command
print (stm_driver('echo'))
