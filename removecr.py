import os
for filename in os.listdir(os.getcwd()):
	if filename!='.git':
		with open(filename, 'rb+') as f:
		    content = f.read()
		    f.seek(0)
		    f.write(content.replace(b'\r', b''))
		    f.truncate()
