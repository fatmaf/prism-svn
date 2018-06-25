import os
import sys
import subprocess

def get_files_by_file_size(dirname, name_hint,ext,size_limit=0,reverse=False):
    """ Return list of file paths in directory sorted by file size """

    # Get list of files
    filepaths = []
    for basename in os.listdir(dirname):
        if name_hint in basename:
            filename = os.path.join(dirname, basename)
            if os.path.isfile(filename):
                if filename.endswith(ext):
                    filepaths.append(filename)

    # Re-populate list with filename, size tuples
    for i in xrange(len(filepaths)):
        filepaths[i] = (filepaths[i], os.path.getsize(filepaths[i]))

    # Sort list by file size
    # If reverse=True sort from largest to smallest
    # If reverse=False sort from smallest to largest
    filepaths.sort(key=lambda filename: filename[1], reverse=reverse)

    size_limited_filepaths = []
    # Re-populate list with just filenames
    for i in xrange(len(filepaths)):
        #print filepaths[i]
        #raw_input("pak")
        if size_limit > 0:
            if filepaths[i][1] < size_limit:
                size_limited_filepaths.append(filepaths[i][0])
        else:
            size_limited_filepaths.append(filepaths[i][0])
            

    return size_limited_filepaths


def get_files_by_file_size_cwd(name_hint,ext,size_limit=512000):
    cwd = os.getcwd()
    print "Retrieving :"+cwd+" "+name_hint+" "+ext+" limit:"+str(size_limit)
    raw_input("continue")
    files = get_files_by_file_size(cwd,name_hint,ext,size_limit)
    #print files
    return files

def do_bash_command(bashCommand,doBackground = True):
    if not doBackground:
        process = subprocess.Popen(bashCommand.split(),stdout=subprocess.PIPE)
        output,error = process.communicate()
    else:
        process = subprocess.Popen(bashCommand.split())
    
    
def convert_to_dot(filename,open=True):
    print "Converting "+filename
    bash_command = "dot -Tsvg "+filename+" -o "+filename+".svg"
    do_bash_command(bash_command,False)
    if open:
        print "Opening "+filename+".svg"
        next_command = "google-chrome-stable "+filename+".svg"
        do_bash_command(next_command)

def convert_all_to_dot(files,open=True):
    for filename in files:
        convert_to_dot(filename)
        
    print "\n\nDone"

if __name__=="__main__":
    print "Usage: python convert_dots_to_svg.py name_hint size_limit(in bytes) ext"
    name_hint = "topo"
    ext="dot"
    size_limit = 512000
    if len(sys.argv) == 1:
        print "Using default name hint "+name_hint+" ext "+ext+" size "+str(size_limit)+"bytes"
    else:
        if len(sys.argv)>=2:
            name_hint = sys.argv[1]
        if len(sys.argv)>=3:
            size_limit = int(sys.argv[2])
        if len(sys.argv)>=4:
            ext = sys.argv[3]
        if len(sys.argv)>=5:
            print "ignoring other arguments"
        print "Using default name hint "+name_hint+" ext "+ext+" size "+str(size_limit)+"bytes"
        
            
            
    files=get_files_by_file_size_cwd(name_hint,ext,size_limit)
    convert_all_to_dot(files)
