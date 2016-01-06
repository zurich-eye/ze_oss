#!/usr/bin/python
"""
@author: Christian Forster
"""

import os
import yaml
import numpy as np
import argparse
import rospkg
from tabulate import tabulate
import matplotlib.pyplot as plt
import seaborn.apionly as sns # https://stanford.edu/~mwaskom/software/seaborn/tutorial/color_palettes.html
from jinja2 import Environment, FileSystemLoader # http://pbpython.com/pdf-reports.html
from weasyprint import HTML
from matplotlib import rc
rc('font',**{'family':'serif','serif':['Cardo']})
rc('text', usetex=True)

FORMAT='.pdf'

def print_tables_to_pdf(comparison_dir, tables, title):
    """
    tables must be a list of dictionaries that contain 'title' and 'data'
    """
    # print to file
    template_dir = os.path.join(rospkg.RosPack().get_path('svo_analysis'), 
                                'src/svo_analysis/templates')
    env = Environment(loader=FileSystemLoader(template_dir))
    template = env.get_template("report.html")
    template_vars = dict()
    template_vars['title'] = title
    template_vars['tables'] = tables
    html_out = template.render(template_vars)
    HTML(string=html_out).write_pdf(os.path.join(comparison_dir, 'report.pdf'))


def compute_average_table(table):
    """
    the first column in the table always contains the label
    """
    
    # find unique labels
    summarized_table = dict() # label -> list of data
    for row in table:
        label = row[0]
        data = row[1:]
        if label not in summarized_table:
            summarized_table[label] = list()
        summarized_table[label].append(data)
        
    averaged_table = list()
    for key, data in summarized_table.iteritems():
        data = np.array(data)
        mean = np.mean(data,0)
        std  = np.std(data,0)
        new_data = [key]
        for i in range(len(mean)):
            new_data.append('{:.4f} +/- {:.4f}'.format(mean[i], std[i]))
        averaged_table.append(new_data)
            
    return averaged_table
        
    
def statistics_table(label, comparison_dir, trace_dirs):
    
    header = ['', label+' Mean', label+' Median', label+' RMSE', 'Sdt', '# Samples']
    table = list()
    for trace_dir in trace_dirs:
        params_filename = os.path.join(trace_dir, 'params.yaml')
        stats_filename  = os.path.join(trace_dir, 'analysis_statistics.yaml')
        assert os.path.exists(params_filename)
        assert os.path.exists(stats_filename)
        params = yaml.load(open(params_filename,'r'))
        
        if not os.path.exists(stats_filename):
            print('ERROR: ' + stats_filename + ' does not exist!')
            continue
        
        stats = yaml.load(open(stats_filename,'r'))

        if label not in stats:
            print('ERROR: ' +label+ ' not in '+trace_dir)
            continue
        if 'mean' not in stats[label]:
            print('ERROR: ' +label+ ' in '+trace_dir + ' has no data!')
            continue
        
        data = [params['experiment_label'],
                stats[label]['mean'],
                stats[label]['median'],
                stats[label]['rmse'],
                stats[label]['std'],
                stats[label]['num_samples']]
            
        table.append(data)
    print(tabulate(table, headers=header))
    
    trans_table_avg = compute_average_table(table)
    print(tabulate(trans_table_avg, headers=header))

    return tabulate(table, header, tablefmt='html'), tabulate(trans_table_avg, header, tablefmt='html')
    
def print_table(comparison_dir, trace_dirs):

    tables = list()

    # Translation
    trans_table, trans_table_avg = statistics_table('trans', comparison_dir, trace_dirs)
    tables.append({'title': 'Translation Error - Averaged', 'data': trans_table_avg})
    tables.append({'title': 'Translation Error', 'data': trans_table})  
    
    # Timing
    timing_table, timing_table_avg = statistics_table('timing_ms', comparison_dir, trace_dirs)
    tables.append({'title': 'Timing - Averaged', 'data': timing_table_avg})
    tables.append({'title': 'Timing', 'data': timing_table})  
    
    # CPU
    cpu_table, cpu_table_avg = statistics_table('cpu_usage', comparison_dir, trace_dirs)
    tables.append({'title': 'CPU - Averaged', 'data': cpu_table_avg})
    #tables.append({'title': 'CPU', 'data': cpu_table})  
    
    # Memory
    mem_table, mem_table_avg = statistics_table('memory_usage', comparison_dir, trace_dirs)
    tables.append({'title': 'Memory - Averaged', 'data': mem_table_avg})
    #tables.append({'title': 'Memory', 'data': mem_table})  
                        
    print_tables_to_pdf(comparison_dir, tables, 'Report')



    
def set_colors(ax, n):
    ax.set_color_cycle(sns.color_palette("deep", n))    
    
def plot_trajectory(trace_dir, experiments):
    fig_traj_top = plt.figure(figsize=(8,8))
    fig_traj_side = plt.figure(figsize=(8,3))
    ax_traj_top = fig_traj_top.add_subplot(111, aspect='equal', xlabel='x [m]', ylabel='y [m]')
    ax_traj_side = fig_traj_side.add_subplot(111, aspect='equal', xlabel='x [m]', ylabel='z [m]')
    set_colors(ax_traj_top, len(experiments))    
    set_colors(ax_traj_side, len(experiments))    
    ax_traj_top.xaxis.grid(ls='--', color='0.7')    
    ax_traj_top.yaxis.grid(ls='--', color='0.7')
    ax_traj_side.xaxis.grid(ls='--', color='0.7')    
    ax_traj_side.yaxis.grid(ls='--', color='0.7')
    
    for i, exp in enumerate(experiments):
        D = yaml.load(open(os.path.join(exp, 'params.yaml'), 'r'))
        data = np.genfromtxt(os.path.join(exp, 'trajectory.csv'), delimiter=',', skip_header=1, dtype=np.float64)
        ax_traj_top.plot(data[:,0], data[:,1], linestyle='-', label=D['experiment_label'])
        ax_traj_side.plot(data[:,0], data[:,2], linestyle='-')
        #if i == 0:
        #    ax_traj_top.plot(data[:,3], data[:,4], color='black', linestyle='-', label='Groundtruth')
        #    ax_traj_side.plot(data[:,3], data[:,5], color='black', linestyle='-')
        
    fig_traj_top.tight_layout()
    fig_traj_side.tight_layout()
    ax_traj_top.legend()
    fig_traj_top.savefig(os.path.join(trace_dir, 'trajectory_comparison_top'+FORMAT), bbox_inches="tight")
    fig_traj_side.savefig(os.path.join(trace_dir, 'trajectory_comparison_side'+FORMAT), bbox_inches="tight")
    
    
def plot_translation_errors(trace_dir, experiments):
    fig_trans_error = plt.figure(figsize=(6,5))
    ax_trans_error_x = fig_trans_error.add_subplot(311, xlabel='Distance [m]', ylabel='x-Error [m]')
    ax_trans_error_y = fig_trans_error.add_subplot(312, xlabel='Distance [m]', ylabel='y-Error [m]')
    ax_trans_error_z = fig_trans_error.add_subplot(313, xlabel='Distance [m]', ylabel='z-Error [m]')
    ax_trans_error_x.yaxis.grid(ls='--', color='0.7')
    ax_trans_error_y.yaxis.grid(ls='--', color='0.7')
    ax_trans_error_z.yaxis.grid(ls='--', color='0.7')
    set_colors(ax_trans_error_x, len(experiments))    
    set_colors(ax_trans_error_y, len(experiments))    
    set_colors(ax_trans_error_z, len(experiments))    
    
    for i, exp in enumerate(experiments):
        D = yaml.load(open(os.path.join(exp, 'params.yaml'), 'r'))
        data = np.genfromtxt(os.path.join(exp, 'translation_error.csv'), delimiter=',', skip_header=1, dtype=np.float64)
        ax_trans_error_x.plot(data[:,0], data[:,1], linestyle='-', label='r'+D['experiment_label'])
        ax_trans_error_y.plot(data[:,0], data[:,2], linestyle='-', label='r'+D['experiment_label'])
        ax_trans_error_z.plot(data[:,0], data[:,3], linestyle='-', label='r'+D['experiment_label'])
        
    fig_trans_error.tight_layout()
    ax_trans_error_x.legend()
    fig_trans_error.savefig(os.path.join(trace_dir, 'translation_error'+FORMAT), bbox_inches="tight")
    
def plot_relative_errors(trace_dir, experiments):
    fig_relative_errors = plt.figure(figsize=(6,5))
    ax_rel_1 = fig_relative_errors.add_subplot(311, xlabel='Measurement', ylabel='Translation-Error [m/s]')
    ax_rel_2 = fig_relative_errors.add_subplot(312, xlabel='Measurement', ylabel='Yaw-Error [deg/s]')
    ax_rel_3 = fig_relative_errors.add_subplot(313, xlabel='Measurement', ylabel='Roll/Pitch-Error [deg/s]')
    ax_rel_1.yaxis.grid(ls='--', color='0.7')
    ax_rel_2.yaxis.grid(ls='--', color='0.7')
    ax_rel_3.yaxis.grid(ls='--', color='0.7')
    set_colors(ax_rel_1, len(experiments)) 
    set_colors(ax_rel_2, len(experiments)) 
    set_colors(ax_rel_3, len(experiments)) 
    
    for i, exp in enumerate(experiments):
        D = yaml.load(open(os.path.join(exp, 'params.yaml'), 'r'))
        data = np.genfromtxt(os.path.join(exp, 'relative_error.csv'), delimiter=',', skip_header=2, dtype=np.float64)
        ax_rel_1.plot(data[:,0], linestyle='-', label=D['experiment_label'])
        ax_rel_2.plot(data[:,2], linestyle='-', label=D['experiment_label'])
        ax_rel_3.plot(data[:,3], linestyle='-', label=D['experiment_label'])
        
    fig_relative_errors.tight_layout()
    ax_rel_1.legend()
    fig_relative_errors.savefig(os.path.join(trace_dir, 'relative_errors'+FORMAT), bbox_inches="tight")
    
    
def plot_rms_errors(trace_dir, experiments):
    fig_relative_errors = plt.figure(figsize=(6,4))
    ax_rms_1 = fig_relative_errors.add_subplot(211, ylabel='Pos. RMSE [m]')
    ax_rms_2 = fig_relative_errors.add_subplot(212, xlabel='Distance [m]', ylabel='Rot. RMSE [deg]')
    ax_rms_1.yaxis.grid(ls='--', color='0.7')
    ax_rms_2.yaxis.grid(ls='--', color='0.7')
    set_colors(ax_rms_1, len(experiments)) 
    set_colors(ax_rms_2, len(experiments)) 
    max_x = 0
    for i, exp in enumerate(experiments):
        D = yaml.load(open(os.path.join(exp, 'params.yaml'), 'r'))
        data_trans = np.genfromtxt(os.path.join(exp, 'translation_error.csv'), delimiter=',', skip_header=1, dtype=np.float64)
        data_rot = np.genfromtxt(os.path.join(exp, 'orientation_error.csv'), delimiter=',', skip_header=1, dtype=np.float64)
        rms_trans = np.sqrt(np.sum(data_trans[:,1:4]**2,1))
        rms_rot   = np.sqrt(np.sum(data_rot[:,1:4]**2,1))
        ax_rms_1.plot(data_trans[:,0], rms_trans, linestyle='-', label=D['experiment_label'])
        ax_rms_2.plot(data_trans[:,0], rms_rot,   linestyle='-')
        max_x = max(max_x, data_trans[-1,0])
    fig_relative_errors.tight_layout()
    #ax_rms_1.legend(loc='upper left')
    ax_rms_1.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
                    ncol=1, mode="expand", borderaxespad=0.)
    ax_rms_1.set_xlim([0, max_x])
    ax_rms_2.set_xlim([0, max_x])
    fig_relative_errors.savefig(os.path.join(trace_dir, 'rms_errors'+FORMAT), bbox_inches="tight")
    
if __name__=='__main__':
    
    # parse command line
    parser = argparse.ArgumentParser(description='''
    Analyse trajectory
    ''')
    parser.add_argument('--trace_dir', help='folder with the results', default='')
    parser.add_argument('--reference', help='', type=str, default="")
    args = parser.parse_args()
    
    ###############################################################################    
    
    trace_dir = args.trace_dir
    experiments = [experiment for experiment in args.reference.split(',')]
    experiments.insert(-1, args.trace_dir)
    

    