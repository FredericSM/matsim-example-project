import gzip

file_path = 'output/ITERS/it.10/10.events.xml.gz'
with gzip.open(file_path, 'rb') as f:
    data = f.read()

print(data)

# Now you can process the 'data' variable containing the contents of the .gz file
