import csv

fname = "lab2_traj_log.csv"  # change to your filename

with open(fname, newline="") as f:
    r = csv.DictReader(f)
    rows = list(r)

print("Rows:", len(rows))
print("Columns:", rows[0].keys())

for i in [0, 1, 2, -3, -2, -1]:
    row = rows[i]
    print(
        i,
        "est:", row["x_est"], row["y_est"],
        "gt:",  row["x_gt"], row["y_gt"]
    )
