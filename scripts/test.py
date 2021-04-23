#!/usr/bin/env python

import sys 
import os
import time 
import numpy as np
from collections import OrderedDict
from operator import getitem


class blueprint:

	def __init__(self):
		
		self.color = OrderedDict({
		    'blue': {'name':'entrance', 'location': [1,1], 'index': 1},
		    'red': {'name':'closet', 'location': None, 'index': 2},
		    'green': {'name':'living room', 'location': None, 'index': 3},
		    'yellow': {'name':'kitchen', 'location': None, 'index': 4},
		    'magenta': {'name':'bathroom', 'location': None, 'index': 5},
		    'black': {'name':'bedroom', 'location': None, 'index': 6},
		})
            
        	self.findCounter = 0

   	def preFind(self):

		res = OrderedDict(sorted(self.color.items(),
			key = lambda x: getitem(x[1], 'index')))
		
		locations = []
		known = []
        	
		if self.findCounter == 0:
            		self.findCounter +=1
            		return [-5, 8]
        
        	for key, value in res.items():
			#print('------KEY: ', key)
			#print('******VAL: ', value['location'])
			locations.append(value['location'])
		for loc in locations:
			if loc is None:
				break
			known.append(loc)
		self.findCounter +=1
		return known[-1]


if __name__ == '__main__' :
    
    rm = blueprint() 
    go = rm.preFind()
    print(go[0], 'and', go[1])
    print('##################')
    rm.color['red'].update( location = [2,2])
    go = rm.preFind()
    print(go[0], 'and', go[1])
    print('##################')
    rm.color['green'].update( location = [3,3])
    go = rm.preFind()
    print(go[0], 'and', go[1])
    print('##################')
    rm.color['yellow'].update( location = [4,4])
    go = rm.preFind()
    print(go[0], 'and', go[1])
    print('##################')
