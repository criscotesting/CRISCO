import random, copy
import itertools

import numpy as np
from scipy import stats

from CRISCO.generate_scenario.TrafficFactors.Environment import get_environment

class utils2 :
     def product(self, allparams):
         newlist=[]
         for x in eval('itertools.product'+str(tuple(allparams))):
             newlist.append(x)
         return newlist

     def get_pairslist(self, productedlist, pairlen):
         pwlist = []
         for i in productedlist:
             subtemplist = []
             for sublista in itertools.combinations(i, pairlen):
                 subtemplist.append(sublista)
             pwlist.append(subtemplist)
         return pwlist

     def pairwise(self, allparams, pairlen):
         productedlist=self.product(allparams)
         listb = self.get_pairslist(productedlist, pairlen)
         sublistlen = len(listb[1])
         flag = [0]*sublistlen
         templistb = copy.deepcopy(listb)
         delmenu = []
         holdmenu=[]
         for cow in listb:
             for column in cow:
                 for templistbcow in templistb:
                     Xa = cow.index(column)
                     Ya = listb.index(cow)
                     if templistbcow != cow and column == templistbcow[Xa]:
                         flag[Xa] = 1
                         break
                     else:
                         flag[Xa] = 0
             if 0 not in flag:
                 num = listb.index(cow)
                 delmenu.append(num)
                 templistb.remove(cow)
             else:
                 num2 = listb.index(cow)
                 holdmenu.append(num2)
         return self.pwresult(productedlist, holdmenu)

     def pwresult(self, slist, holdmenu):
         holdparamslist = []
         for  item in holdmenu:
             holdparamslist.append(slist[item])
         return holdparamslist

def f_x(x):
    return 1/(1 + np.exp(-x))

def distribution(mu=0, sigma=1):
    distribution = stats.norm(mu, sigma)
    return distribution

def importance_sampling(mu, n):
    mu_target = mu
    sigma_target = 1
    mu_appro = 1
    sigma_appro = 1

    p_x = distribution(mu_target, sigma_target)
    q_x = distribution(mu_appro, sigma_appro)

    s = 0
    for i in range(n):
        # draw a sample
        x_i = np.random.normal(mu_target, sigma_target)
        s += f_x(x_i)

    value_list = []
    for i in range(n):
        x_i = np.random.normal(mu_appro, sigma_appro)
        value = f_x(x_i) * (p_x.pdf(x_i) / q_x.pdf(x_i))

        value_list.append(value)

    return value_list

def pairwise_environment_parameters():
    parameters, domains = get_environment()
    # pairwise tesing
    u2 = utils2()
    finallist = u2.pairwise(domains, 2)
    size = len(finallist)
    index = random.randint(0, size-1)
    parameter_list = finallist[index]

    return parameters, parameter_list
