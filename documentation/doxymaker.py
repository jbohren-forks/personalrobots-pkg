import sys, os, tempfile
from subprocess import Popen, PIPE

TEMPLATE_FILE = 'doxy.template'

#$INPUT, $OUTPUT_DIRECTORY

#######################################################################
# initialization

try:
    rosroot = os.environ['ROS_ROOT']
except KeyError:
    print >> sys.stderr, "Please set ROS_ROOT before running"
    sys.exit(1)

# load our stock template file, which we will customize per package
if not os.path.isfile(TEMPLATE_FILE):
    print >> sys.stderr, "Cannot locate doxygen template file '%s'"%TEMPLATE_FILE
    sys.exit(1)
    
f = open(TEMPLATE_FILE, 'r')
try:
    doxyTemplate = f.read()
finally:
    f.close()


# use 'rospack list' to locate all packages and store their paths in a dictionary
rospackList = (Popen(['rospack', 'list'], stdout=PIPE).communicate()[0] or '').strip().split('\n')
rospackList = [x.split(' ') for x in rospackList if ' ' in x]

packages = {}
for package, path in rospackList:
    packages[package] = path

#######################################################################
# Crawl manifest.xml dependencies

try:
    rostoolsDir = (Popen(['rospack', 'find', 'rostools'], stdout=PIPE).communicate()[0] or '').strip()
    sys.path.append(os.path.join(rostoolsDir, 'scripts'))
    import manifest, packspec

    for package, path in packages.iteritems():
        f = os.path.join(path, manifest.MANIFEST_FILE)
        #m = manifest.parseFile(f)
        #TODO: parse manifest files in each package
        #I'm waiting for a decision on our latest manifest spec WRT required fields
        #print m.depends
        
except ImportError, e:
    print >> sys.stderr, "ERROR: Cannot locate rostools package, documentation will not include links to dependencies"


#######################################################################
# Generate Doxygen

def createPackageTemplate(doxyTemplate, package, path, htmlPaths):
    #replace vars in the template file to point to package we are documenting
    t = doxyTemplate.replace('$INPUT', path)
    t = t.replace('$PROJECT_NAME', package)
    t = t.replace('$OUTPUT_DIRECTORY', os.path.join(path, 'doc'))
    # although HTML_OUTPUT is fine being relative, it's worth actually using a template
    # variable for it given that we depend on being able to compute it
    htmlOutput = os.path.join(path, 'doc', 'html')
    t = t.replace('$HTML_OUTPUT', htmlOutput)
    htmlPaths[package] = htmlOutput
    return t

success = []
htmlPaths = {}
try:
    for package, path in packages.iteritems():
        t = tempfile.NamedTemporaryFile('w+')
        try:
            pDoxyTemplate = createPackageTemplate(doxyTemplate, package, path, htmlPaths)
            f = open('test.template', 'w+')
            t.write(pDoxyTemplate)
            t.flush()
            command = ['doxygen', t.name]
            print "doxygen-ating %s"%package
            if 1:
                _, stderr = Popen(command, stdout=PIPE, stderr=PIPE).communicate()
                if stderr:
                    print >> sys.stderr, stderr
            success.append(package)
        finally:
            t.close()
finally:
    pass

    
#######################################################################
# Generate Documentation Index

def computeRelative(src, target):
    s1, s2 = [os.path.realpath(p).split(os.sep) for p in [src, target]]
    #filter out empties
    s1, s2 = filter(lambda x: x, s1), filter(lambda x: x, s2)
    i = 0
    while i < min(len(s1), len(s2)):
        if s1[i] != s2[i]:
            break
        i+=1
    rel = ['..' for d in s1[i:]] + s2[i:]
    return os.sep.join(rel)

f = open('index.html', 'w')
try:
    #TODO: replace with a doxygen-based header/footer
    header = """
<html><head><title>ROS Packages Documentation</title></head><body>
<h1>ROS Packages Documentation</h1>
<ul>
"""
    footer = "</ul></body></html>"
    def caselessComp(x, y):
        if x.lower() < y.lower():
            return -1
        return 1
    success.sort(caselessComp)
    list = ['<li><a href="%s/index.html">%s</a></li>'%(computeRelative("./", htmlPaths[p]), p) for p in success]
    f.write(header)
    f.write('\n'.join(list))
    f.write(footer)
finally:
    f.close()
