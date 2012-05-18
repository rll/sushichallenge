class Hi(object):
    a = 2
    @classmethod
    def printa(kls):
        print kls, kls.a
    @staticmethod
    def sayyo():
        print "yo"

class HiB(Hi):
    pass

Hi.printa()
Hi.sayyo()
print dir(HiB)