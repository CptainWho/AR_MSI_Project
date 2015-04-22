__author__ = 'Ecki'


def staticvartest():
    staticvartest.count += 0.1
    print staticvartest.count
staticvartest.count = 0

for i in range(10):
    staticvartest()