# Adapted from https://datascience.blog.wzb.eu/2016/08/12/a-tip-for-the-impatient-simple-caching-with-python-pickle-and-decorators/

from functools import wraps
import hashlib
import inspect
import os
import pickle
from time import perf_counter
from warnings import warn

_caching_enabled = True

def disable_caching():
    _caching_enabled = False

def cache_result(fn):
    '''
    A decorator which caches the result of the function call with a given set of
    arguments to disk.

    Be careful when using this! There are a number of gotchas:
     - If you change a function's definition after caching a result, you will
       need to delete to cached pkl files if you don't want to reload an
       outdated result
     - If the function is non-deterministic or relies on variables which are not
       passed in as arguments you'll get spurious caching
     - See note about Jupyter Notebooks below
    '''
    @wraps(fn)
    def wrapped(*args, **kwargs):
        # Serialise the arguments and take the SHA256 hash
        arg_hash = hashlib.sha256(pickle.dumps((args, kwargs))).hexdigest()

        # There doesn't seem to be an easy way to get the current notebook's name :-(
        #
        # This means you can get collisions if you're passing the same arguments
        # to functions called the same thing in different notebooks.
        filename = inspect.stack()[1].filename
        if filename.startswith('<ipython-input'):
            filename = 'ipython'
        else:
            filename = os.path.basename(filename)

        CACHE_PATH = '.bob_robotics_cache'
        if not os.path.exists(CACHE_PATH):
            os.mkdir(CACHE_PATH)
        cachefile = os.path.join(CACHE_PATH, '%s_%s_%s.pkl' % (filename, fn.__name__, arg_hash))

        # if cache exists -> load it and return its content
        if os.path.exists(cachefile):
            if _caching_enabled:
                with open(cachefile, 'rb') as cachehandle:
                    print("Using cached result from '%s'" % cachefile)
                    res, elapsed = pickle.load(cachehandle)
                    print('%s() took %g s to run (without caching)' % (fn.__name__, elapsed))
                    return res
            else:
                warn('Cache file exists, but caching has been disabled')

        # execute the function with all arguments passed
        print('Starting %s()...' % fn.__name__)
        t0 = perf_counter()
        res = fn(*args, **kwargs)
        elapsed = perf_counter() - t0
        print('%s() took %g s to run (without caching)' % (fn.__name__, elapsed))

        # write to cache file
        with open(cachefile, 'wb') as cachehandle:
            print("Saving result to cache '%s'" % cachefile)
            pickle.dump((res, elapsed), cachehandle)

        return res

    return wrapped
