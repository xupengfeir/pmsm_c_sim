#coding:u8
from pylab import plt, mpl, arange, show
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
# from pprint import pprint
from collections import OrderedDict as O
# plot style
plt.style.use('ggplot') 


mpl.rcParams['legend.fontsize'] = 14
mpl.rcParams['font.family'] = ['Times New Roman']
mpl.rcParams['font.size'] = 14
# fontdict
font = {'family' : 'Times New Roman',
        'color' : 'darkblue',
        'weight' : 'normal',
        'size' : 14,}


######################
# Read in Data
import csv

try:
    f_name = './pmsm_sim.dat'
    with open(f_name, mode='r') as f:
        print('found '+f_name)
except:
    f_name = '../pmsm_sim.dat'    
print('[Python] Read in data...')

with open(f_name, mode='r') as f:
    buf = f.readlines()
    reader = csv.reader(buf)
    ll = [ [] for _ in range(10) ]  # 创建10个列表
    for idx, row in enumerate(reader):
        if idx == 0:
            continue

        try:
            for ind, el in enumerate(row):
                ll[ind].append(float(el))
        except:
            break

print('\tQuantities amount:', len(ll))
print('\tData Length:', ll[0].__len__())

######################
# Plotting

def get_axis(cNr):
    fig, axes = plt.subplots(ncols=cNr[0], nrows=cNr[1], sharex=True, figsize=(16*0.8, 9*0.8), dpi=80, facecolor='w', edgecolor='k');
    fig.subplots_adjust(right=0.95, bottom=0.1, top=0.95, hspace=0.2, wspace=0.02)    
    if sum(cNr)<=2:
        return axes
    else:
        return axes.ravel()

def plot_it(ax, ylabel, d):
    count = 0
    for k, v in d.items():
        if count == 0:
            count += 1
            ax.plot(time, v, '-', lw=1)
        else:
            ax.plot(time, v, '-', lw=1)

    ax.set_ylabel(ylabel, fontdict=font)


#################################
time = arange(1,ll[0].__len__()+1,1) * 10/4000.000000
ax_list = get_axis((1,5))
plot_it(ax_list[0], r'$i_s$ [A]', O([
                                             (r'0',   ll[0]),  
                                             (r'1',   ll[1]),  
                                             ]))
plot_it(ax_list[1], r'$\omega$ [rpm]', O([
                                             (r'0',   ll[2]),  
                                             ]))
plot_it(ax_list[2], r'$\theta_{rm} $ [rad]', O([
                                             (r'1',   ll[3]),  
                                             ]))
plot_it(ax_list[3], r'voltage [V]', O([
                                             (r'0',   ll[4]),  
                                             (r'1',   ll[5]),  
                                             ]))
plot_it(ax_list[4], r'DQ current [A]', O([
                                             (r'1',   ll[6]),  
                                             (r'2',   ll[7]),  
                                             (r'3',   ll[8]),  
                                             (r'4',   ll[9]),  
                                             ]))

plt.show()




