import pandas as pd

# 첫 번째 파일 확인
df = pd.read_csv('make_csv/agent_0.csv')
print("Column names:", df.columns.tolist())
print("\nFirst few rows:")
print(df.head())