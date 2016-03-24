
# file_content = open("raw2.txt", 'r').read()
# parsed = ""
# parsed_file = open("parsed2.txt", 'w+')
# index = 0
# for row in file_content.splitlines():
#     if len(row) > 0 and (index % 2) != 0:
#         parsed += row.replace("\t", ",") + "\n"
#     index += 1
# parsed_file.write(parsed)

file_content = open("kalman_data3.txt", 'r').read()
parsed = ""
parsed_file = open("kalman_data3_parsed.txt", 'w+')
for row in file_content.splitlines():
    if len(row) > 0 and row[0:2] == "40":
        parsed += row.replace(" ", ",") + "\n"
parsed_file.write(parsed)
