# =============================================== #
#                                                 #
#     Singleton implementation for Singleton      #
#     patterns in the Interoperability code.      #
#                                                 #
#             Version : 2.1.4                     #
#              Author : David Kroell              #
#                                                 #
#            (C) Aridine Technologies             #
#                                                 #
# =============================================== #


def singleton(c):
    """
    Singleton definition for classes.

    Parameters:
    ----------
    c: class
        Class that is being issued for singleton definition.

    instances: object
        return a currently active instance of the class or a new instance if one hasn't been created.
    """

    __Instances__ = {}

    def get_instance(*args):
        """ Static access method for the current instance """
        
        if c not in __Instances__:
            __Instances__[c] = c(*args)

        return __Instances__[c]
    
    return get_instance


@singleton
class TestClass:
    def __init__(self):
        """
        This is for demonstration purposes
        Just to use as a testing file.
        """

        self.count = 0
    
    def inc(self):
        """
        increment the count variable for the test class.
        """
        
        self.count += 1


def test():
    """
    Instantiate two seperate instances of testclass
    """
    
    test1 = TestClass()
    test2 = TestClass()

    test1.inc()

    if not(test1.count == test2.count):
        return False
    else:
        return True


if __name__ == "__main__":
    result = test()
    print("Singleton class testing: Result: ", result)
