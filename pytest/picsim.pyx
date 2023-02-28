# cython: profile=False
# cython: embedsignature = True
# cython: language_level = 3
# distutils: language = c++

# C-Import Cython Definitions

from libc.stdint cimport uint64_t, uint32_t, uint16_t, uint8_t
from libc.stdlib cimport calloc, malloc, free
from libc.string cimport memset, strcmp
from libc.stdio cimport printf

from libcpp cimport bool
from libcpp.memory cimport unique_ptr, shared_ptr, make_shared, allocator
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp.utility cimport pair

from cpython cimport array
from cpython.ref cimport PyObject
from cpython.mem cimport PyMem_Malloc, PyMem_Realloc, PyMem_Free
from cpython.bytes cimport PyBytes_FromStringAndSize

from cython.operator cimport dereference as deref

from enum import IntEnum

# Import Python Modules

import array
import numbers

# C-Import Custom Cython Definitions

cimport cpicsim

P16F84A   = cpicsim.P16F84A
P16F777   = cpicsim.P16F777
P16F18855 = cpicsim.P16F18855
P18F452   = cpicsim.P18F452

def getprocbyname(name : str) -> int:
    return cpicsim.getprocbyname(name.encode('utf8'))

def getfprocbyname(name : str) -> int:
    return cpicsim.getfprocbyname(name.encode('utf8'))

def getproclist():
    proclist = []
    cdef char[25][30] plist
    pc = cpicsim.getproclist(plist, cpicsim.PMAX)
    cdef const char * pname
    for i in range(pc):
        pname = &plist[i][0]
        proclist.append(pname.decode('utf8'))
    return proclist

cdef class pic_:
    cdef cpicsim._pic pic
    def __cinit__(self):
        pass
    def __dealloc__(self):
        pass
    def set_serial(self, nser : int, name : str, flowcontrol : int, ctspin : int, rtspin : int) -> int:
        return cpicsim.pic_set_serial(&self.pic, nser, name.encode('utf8'), flowcontrol, ctspin, rtspin)
    def init(self, processor : int, fname : str, leeprom : int, freq : float) -> int:
        return cpicsim.pic_init(&self.pic, processor, fname.encode('utf8'), leeprom, freq)
    def reset(self, flags : int) -> int:
        return cpicsim.pic_reset(&self.pic, flags)
    def step(self):
        cpicsim.pic_step(&self.pic)
    def end(self):
        cpicsim.pic_end(&self.pic)
    def get_pin(self, pin : uint8_t) -> uint8_t:
        return cpicsim.pic_get_pin(&self.pic, pin)
    def set_pin(self, pin : uint8_t, value : uint8_t) -> int:
        return cpicsim.pic_set_pin(&self.pic, pin, value)

    @property
    def print(self):
         return True if self.pic.print_ else False
    @print.setter
    def print(self, value : bint):
        self.pic.print_ = 1 if value else 0

class pic(pic_):
    def __init__(self):
        pass
    def __del__(self):
        pass
