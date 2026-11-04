import time

# def routine(number):
#         print(f"Start of Task{number}")
#         # Computation simulation
#         result = 0
#         for i in range(1000000):
#             result += i
#         print(f"End of Task{number}, result: {result}")


print("Test on non-parallelized tasks")
starttime = time.perf_counter()

print("Start of Task1")
result1 = 0
for i in range(1000000):
    result1 += i
print(f"End of Task 1, result: {result1}")

# print("Start of Task2")
# result2 = 0
# for i in range(1000000):
#     result2 += i
# print(f"End of Task 2, result: {result2}")

# routine(1)
# routine(2)

print(time.perf_counter() - starttime)
print("Non-parallelized tasks are complete")
