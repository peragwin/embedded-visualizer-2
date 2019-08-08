from matplotlib import pyplot as plt

file = open("gdb.txt", "r")
data = file.readline()

start = data.index("{")
end = data.index("}")
data = data[start+1:end]

out = []

sp = data.split(", ")
for d in sp:
    v = d.split(" = ")[1]
    v = float(v)
    out.append(v)

out = out[1::2]

for e in out[:10]:
    print(hex(int(e)))

plt.plot(out)
plt.show()