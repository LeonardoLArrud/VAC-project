import time
from multiprocessing import Process


def routine(number):
    print(f"Start of Task{number}")
    # Computation simulation
    result = 0
    for i in range(3000000):
        result += i
    print(f"End of Task{number}, result: {result}")


def routine2(number):
    print(f"Start of Task{number}")
    # Computation simulation
    result = 0
    for i in range(3000000):
        result += i * 2
    print(f"End of Task{number}, result: {result}")


if __name__ == "__main__":
    # Creation of 2 processes
    process1 = Process(target=routine, args=(2,))
    process2 = Process(target=routine, args=(2,))

    # Process startup
    print("Process startup")
    starttime = time.perf_counter()

    process1.start()
    process2.start()

    # Wait for the processes to finish
    process1.join()
    process2.join()

    print(time.perf_counter() - starttime)
    print("The processes are complete")
