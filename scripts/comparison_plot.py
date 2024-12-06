import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math
import os

class Plotter:
    '''
    This class is used to plot comparison plot for optimized and unoptimized paths 
    for DynaPRM.
    '''
    
    def __init__(self, data):
        '''
        Init method for Plotter class.
        Arguments:
            data: csv file containing the data for optimized and unoptimized paths.
            data format : Total exploration time, Total path length, Total obstacle distance
        '''
        self.data = data
        self.df = pd.DataFrame(self.data, columns=['Total Time', 'Path Length', 'Obstacle Distance'])
        self.labels = ['Total Time (s)', 'Path Length (m)', 'Obstacle Distance (m)']
    
    def stats_cal(self):
        '''
        This method is used to calculate the mean and standard deviation of the data.
        '''
        self.mean = np.mean(self.data, axis=0)
        self.std = np.std(self.data, axis=0)
        
    def plot(self, save_path=None):
        '''
        This method is used to plot simple bar graph for the data.
        Arguments:
            save_path: Path to save the plot. If None, plot will be displayed.
        '''
        self.stats_cal()
        
        plt.figure(figsize=(10, 6))
        x = np.arange(len(self.labels))
        width = 0.35
        
        # Create bar plot
        plt.bar(x, self.mean, width, yerr=self.std, capsize=5)
        
        # Customize plot
        plt.xlabel('Metrics')
        plt.ylabel('Values')
        plt.title('DynaPRM Path Metrics')
        plt.xticks(x, self.labels, rotation=45)
        
        # Add value labels on top of bars
        for i, v in enumerate(self.mean):
            plt.text(i, v + self.std[i], f'{v:.2f}±{self.std[i]:.2f}', 
                    ha='center', va='bottom')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path)
        else:
            plt.show()
            
    def plot_comparison(self, other_data, save_path=None):
        '''
        This method is used to plot comparison bar graph between two sets of data.
        Arguments:
            other_data: Second set of data to compare with
            save_path: Path to save the plot. If None, plot will be displayed.
        '''
        plt.figure(figsize=(10, 6))
        x = np.arange(len(self.labels))
        width = 0.35
        
        # Calculate stats for both datasets
        self.stats_cal()
        other_mean = np.mean(other_data, axis=0)
        other_std = np.std(other_data, axis=0)
        
        # Create bar plots
        plt.bar(x - width/2, self.mean, width, yerr=self.std, 
                label='Original', capsize=5)
        plt.bar(x + width/2, other_mean, width, yerr=other_std, 
                label='Optimized', capsize=5)
        
        # Customize plot
        plt.xlabel('Metrics')
        plt.ylabel('Values')
        plt.title('DynaPRM Path Comparison')
        plt.xticks(x, self.labels, rotation=45)
        plt.legend()
        
        # Add value labels
        for i, v in enumerate(self.mean):
            plt.text(i - width/2, v + self.std[i], f'{v:.2f}', 
                    ha='center', va='bottom')
        for i, v in enumerate(other_mean):
            plt.text(i + width/2, v + other_std[i], f'{v:.2f}', 
                    ha='center', va='bottom')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path)
        else:
            plt.show()
            
    def save_stats(self, save_path):
        '''
        This method saves the statistics to a csv file.
        Arguments:
            save_path: Path to save the statistics
        '''
        self.stats_cal()
        stats_df = pd.DataFrame({
            'Metric': self.labels,
            'Mean': self.mean,
            'Std Dev': self.std
        })
        stats_df.to_csv(save_path, index=False)