# Copyright (c) 2013 Ed Kellett

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from __future__ import print_function

from functools import reduce
import __future__
import operator
import string
import sys


_basestring = globals().get('basestring', str)
_future_mask = reduce(operator.or_, [f.compiler_flag for f in [getattr(__future__, k) for k in __future__.all_feature_names]], 0)


class _Formatter(string.Formatter):
    def __init__(self, locals_, globals_, flags=0, verbose=False):
        self.locals = locals_
        self.globals = globals_
        self.flags = flags & _future_mask
        self.verbose = verbose

    def _get_field(self, field_name, args, kwargs):
        try:
            return string.Formatter.get_field(self, field_name, args, kwargs)
        except:
            co = compile(field_name, '<eval>', mode='eval', flags=self.flags)
            return (eval(co, self.locals, self.globals), None)

    def get_field(self, field_name, args, kwargs):
        if self.verbose:
            return ("[{}] => {}".format(field_name, self._get_field(field_name, args, kwargs)[0]), None)
        else:
            return self._get_field(field_name, args, kwargs)


class _UDict(object):
    def __init__(self, *a):
        self.maps = a

    def __getitem__(self, k):
        m = list(self.maps)
        while m:
            try:
                return m.pop(0)[k]
            except KeyError:
                continue
        raise KeyError(k)

    def __contains__(self, k):
        return any(k in d for d in self.maps)


class _Interpolate(object):
    def __init__(self, is_verbose):
        self.is_verbose = is_verbose

    def interpolate(self, str_, frame=None):
        return self._interpolate(str_, depth=2, frame=frame)

    def _interpolate(self, str_, depth=2, frame=None):
        frame = frame or sys._getframe(depth)
        code = frame.f_code
        args = [frame.f_locals.get(f)
                for f in code.co_varnames[:code.co_argcount]]
        if code.co_flags & 0x04:
            if code.co_flags & 0x08:
                offset = 1
            else:
                offset = 0
            args.extend(frame.f_locals[code.co_varnames[-1 - offset]])
        fmt = _Formatter(frame.f_locals, frame.f_globals, code.co_flags,
                         verbose=self.is_verbose)
        m = _UDict(frame.f_locals, frame.f_globals)
        return fmt.vformat(str_, tuple(args), m)

    def __mod__(self, other):
        if isinstance(other, _basestring):
            return self._interpolate(other, 2)
        else:
            return NotImplemented

    def __call__(self, fmt):
        return self._interpolate(fmt, 2)

i = _Interpolate(False)
i.verbose = _Interpolate(True)
