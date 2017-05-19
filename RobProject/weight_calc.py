
def estimateCost(filename):
    word = []
    wanted  = "{"
    with open(filename, "r") as file:
        for line in file:
            left,sep,right = line.partition("{")
            if sep:
                word.append(right[:-2].split(','))
                q0 = float(word[0][0])
                q1 = float(word[0][1])
                q2 = float(word[0][2])
                q3 = float(word[0][3])
                q4 = float(word[0][4])
                q5 = float(word[0][5])
                totalCost = 0
    for i in range(1, len(word)):
        q0weight = abs(float(word[i][0]) - q0) * 0.6
        q1weight = abs(float(word[i][1]) - q1) * 0.5
        q2weight = abs(float(word[i][2]) - q2) * 0.4
        q3weight = abs(float(word[i][3]) - q3) * 0.3
        q4weight = abs(float(word[i][4]) - q4) * 0.2
        q5weight = abs(float(word[i][5]) - q5) * 0.1
        q0 = float(word[i][0])
        q1 = float(word[i][1])
        q2 = float(word[i][2])
        q3 = float(word[i][3])
        q4 = float(word[i][4])
        q5 = float(word[i][5])
        print("The cost of ", i, "step is:", q0weight + q1weight + q2weight + q3weight + q4weight + q5weight)
        totalCost += q0weight + q1weight + q2weight + q3weight + q4weight + q5weight
    return totalCost

print( "Original Path Cost:", estimateCost("original.txt"))
print( "Optimized Path Cost:", estimateCost("optimized.txt"))
print( "Optimized and Pruned Path Cost:"), estimateCost("pruned.txt")
