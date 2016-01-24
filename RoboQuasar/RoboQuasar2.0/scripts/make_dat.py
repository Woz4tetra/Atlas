from PIL import Image
import os

FILEDIR = os.path.dirname(os.path.realpath(__file__))

def collect_names(directory="."):
    names = []
    def recurse(directory, names):
        for sub_dir in os.listdir(directory):
            sub_dir = directory + "/" + sub_dir
            if os.path.isdir(sub_dir):
                recurse(sub_dir, names)
            else:
                names.append(sub_dir)
    recurse(directory, names)
    return names

#print os.listdir(".")

#names = []
#for root, subdirs, files in os.walk("face"):
#    names += files
#print names

def pgm_to_png():
    for file_name in collect_names("face"):
        print(file_name)
        image = Image.open(file_name)
        
        extension = file_name.rfind(".")
        file_start = file_name.rfind("/")
        converted_name = "img" + file_name[file_start:extension] + ".jpg"
        image.save(converted_name)
        image.close()

def make_dat(directory="img"):
    dat_file = open("info.dat", "w+")
    for file_name in os.listdir(directory):
        extension = file_name.rfind(".")
        if file_name[extension + 1:] == "png":
            image = Image.open(directory + "/" + file_name)
            width, height = image.size
            dat_line = "%s\t1\t0 0 %s %s\n" % ((directory + "/" + file_name).replace(" ", "\ "),
                                               width, height)
            dat_file.write(dat_line)
    dat_file.close()

def make_neg(directory):
    file_names = os.listdir(directory)
    
    with open("negatives.txt", 'w+') as neg_file:
        for file_name in file_names:
            print(file_name)
            if len(file_name) > 4 and file_name[-4:] == ".png":
                neg_file.write((directory + "/" + file_name).replace(" ", "\ ") + "\r\n")

make_neg("/Users/Woz4tetra/Documents/Self-Driving-Buggy/ben_projects/RoboQuasar1.0/camera/Images/negatives")
