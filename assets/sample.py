import os
import sys
import getopt
from xml.dom.minidom import parse

def main(args):
	opts, args = getopt.getopt(args, 'am:i:')
	file_list = []
	for opt, arg in opts:
		if opt == '-a':
			files = os.listdir('.')
			for file in files:
				if '.xml' in file:
					file_list.append(file)
		elif opt == '-i':
			file_list.append(arg)
		elif opt == '-m':
			if arg == 'cm':
				k = 100
			elif arg == 'm':
				k = 0.01

	for file in file_list:
		DOM_tree = parse(file)
		scene = DOM_tree.documentElement
		strands = scene.getElementsByTagName('Strand')
		for strand in strands:
			particles = strand.getElementsByTagName('particle')
			for particle in particles:
				x = particle.getAttribute('x')
				num_list = [float(ele) * k for ele in x.split()]
				x = '{0} {1} {2}'.format(*num_list)
				particle.setAttribute('x', x)
		with open(file, 'w') as f:
			DOM_tree.writexml(f, encoding='utf-8')

if __name__ == '__main__':
	main(sys.argv[1:])