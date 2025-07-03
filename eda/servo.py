# autoreload 5s
# %load_ext autoreload
# %autoreload 4
# switch to bash
import sqlite3

import pandas as pd

gazebo_data = "../../../ros2_log/pupper_gaz/pupper_gaz_0.db3"
conn = sqlite3.connect(gazebo_data)

tables = pd.read_sql_query("SELECT name FROM sqlite_master WHERE type='table';", conn)

print(tables)

# print topic with id
topics = pd.read_sql_query("SELECT * FROM topics", conn)

# print all without collapsing
pd.set_option('display.max_columns', None)
pd.set_option('display.max_rows', None)
print(topics)