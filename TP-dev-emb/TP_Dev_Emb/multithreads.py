from threading import Thread
import time

def routine(number):
        print(f"Start of Task{number}")
        # Computation simulation
        result = 0
        for i in range(3000000):
            result += i
        print(f"End of Task{number}, result: {result}")

if __name__ == "__main__":
    # Creation of 2 threads
    thread1 = Thread(target=routine, args=(1,))
    thread2 = Thread(target=routine, args=(2,))

    # Thread startup
    print("Thread startup")
    starttime=time.perf_counter()
    
    thread1.start()
    thread2.start()
        
    # Wait for the processes to finish
    thread1.join()
    thread2.join()

    print(time.perf_counter() - starttime)
    print("The threads are complete")
