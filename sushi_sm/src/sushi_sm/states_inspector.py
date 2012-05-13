"""
Author: Lorenzo Riano <lorenzo.riano@gmail.com>
"""
PKG = "sushi_sm"
import roslib; roslib.load_manifest(PKG)

import inspect
import smach

class StateInspector(object):
    """
    Inspects all the smach.State classes in a given module and prepare the right
    arguments to pass to an instance at construction time.
    
    Usage:
    StateInspector(module_name) where module_name has the all the classes to be used.
    """
    def __init__(self, module):
        self.params = {}         
        self.add_module(module)

    def add_module(self, module):
        """Add a module to the list of known modules. Inspects and creates the args.
        """
        sm_classes = ( (cn, co) for (cn,co) in inspect.getmembers(module, inspect.isclass)
                      if issubclass(co, smach.State))
        for class_name, class_obj in sm_classes:
            spec = inspect.getargspec(class_obj.__init__)
            args = [a for a in spec.args if a != 'self']
            self.params[class_obj] = args
            
    def instanciate(self, class_obj, **kwargs):
        """Creates an instance of class_obj using the stored parameters and the additional
        kwargs. 
        
        For a class C that has constructor arguments, it will look up which arguments are
        already in StateInspector __dict__ and it will extend those with kwargs.
        """
        newkwargs = dict( (k,v) for (k,v) in self.__dict__.iteritems() 
                     if k in self.params[class_obj] )
        newkwargs.update(kwargs)
        return class_obj(**newkwargs)
    
    def __call__(self, class_obj, **kwargs):
        return self.instanciate(class_obj, **kwargs)
            
def test():
    import two_arms_states
    insp = StateInspector(two_arms_states)
    insp.world = "world"
    insp.location = "location"
    insp.detector="detector"    
    
    inst = insp.instanciate(two_arms_states.Detect, head_point=(1,2), location=(1,2,3))
    print "Inst: ", inst
    print "Dict: ", inst.__dict__
    
    
    
if __name__ == "__main__":
    test()
            
            