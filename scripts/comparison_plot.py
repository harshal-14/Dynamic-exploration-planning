import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math
import os
import argparse

class Plotter:
    '''
    This class is used to plot comparison plot for paths for DynaPRM.
    '''
    
    def __init__(self, csv_file):
        '''
        Init method for Plotter class.
        Arguments:
            csv_file: Path to the CSV file containing the data for paths.
        '''
        self.df = pd.read_csv(csv_file) # Read the csv to dataframe
        print(f"\nLoaded data from: {csv_file}")
        print("Basic Statistics:")
        print(self.df.describe())
        
        self.labels = ['Total Time (s)',  #Labels lelo bhai
                      'Total Comp Time (s)', 
                      'Total Path Length (m)', 
                      'Total Path Segment', 
                      'Total Distance to Obstacle (m)']
    
    def stats_cal(self):
        '''
        This method is used to calculate the mean and standard deviation of the data.
        '''
        print("\nCalculating detailed statistics...")
        self.mean = np.array([
            self.df['total_time'].mean(), # All colums stats like mean and std
            self.df['total_comp_time'].mean(),
            self.df['total_path_length'].mean(),
            self.df['total_path_segment'].mean(),
            self.df['total_distance_to_obstacle'].mean()
        ])
        
        self.std = np.array([
            self.df['total_time'].std(),
            self.df['total_comp_time'].std(),
            self.df['total_path_length'].std(),
            self.df['total_path_segment'].std(),
            self.df['total_distance_to_obstacle'].std()
        ])
        ## Acche se print dikhna chaiye, user ko samajh aaye
        print("\nDetailed Statistics:")
        for label, mean, std in zip(self.labels, self.mean, self.std):
            print(f"{label}:")
            print(f"  Mean: {mean:.2f}")
            print(f"  Std:  {std:.2f}")
        
    def plot_single(self, save_path=None):
        '''
        This method is used to plot simple bar graph for the data.
        Arguments:
            save_path: Path to save the plot. If None, plot will be displayed.
        '''
        self.stats_cal() # Stats ko calculate karna hai
        plt.figure(figsize=(12, 7)) # Pretty pretty plot bhai
        x = np.arange(len(self.labels))
        width = 0.35 
        bars = plt.bar(x, self.mean, width, yerr=self.std, capsize=5) # Bar plot, just like in the paper
        plt.xlabel('Metrics')
        plt.ylabel('Values')
        plt.title('DynaPRM Path Metrics')
        plt.xticks(x, self.labels, rotation=45, ha='right')
        for i, bar in enumerate(bars): # Add mean and std on top of the bars
            height = bar.get_height()
            plt.text(bar.get_x() + bar.get_width()/2., height + self.std[i],
                    f'{self.mean[i]:.2f}±{self.std[i]:.2f}',
                    ha='center', va='bottom')
        plt.tight_layout()
        
        if save_path: # Hmara Autosaviour bro
            plt.savefig(save_path, bbox_inches='tight', dpi=300)
            print(f"\nSaved plot to: {save_path}")
        plt.show()
                
    def plot_comparison(self, other_data, save_path=None):
        '''
        This method plots comparison showing percentages like in the paper.
        Arguments:
            other_data: DataFrame with comparison data
            save_path: Path to save the plot. If None, plot will be displayed.
        '''
        print("\nGenerating comparison plot...")
        plt.figure(figsize=(8, 6))
        
        unopt_time = self.df['total_time'].mean() # Unoptimized data ka metric
        unopt_length = self.df['total_path_length'].mean() 
        opt_time = other_data['total_time'].mean() # Optimized data ka metric
        opt_length = other_data['total_path_length'].mean()
        
        metrics = ['Expl. Time', 'Path Length'] # paper mei % diya hai, wahi lelo
        baseline = [100, 100]  # Unoptimized baseline
        optimized = [
            (opt_time / unopt_time) * 100,
            (opt_length / unopt_length) * 100
        ]
        
        x = np.arange(len(metrics))
        width = 0.35
        
        plt.bar(x - width/2, baseline, width, label='DEP without Optimization', color='red', alpha=0.9)
        plt.bar(x + width/2, optimized, width, label='DEP with Optimization', color='green', alpha=0.9)
        for i, v in enumerate(baseline):
            plt.text(i - width/2, v, f'{v:.1f}%', 
                    ha='center', va='bottom')
        for i, v in enumerate(optimized):
            plt.text(i + width/2, v, f'{v:.1f}%', 
                    ha='center', va='bottom')
        # Naam daalo properly
        plt.xlabel('Metrics')
        plt.ylabel('Percentage [%]')
        plt.xticks(x, metrics)
        plt.legend()
        plt.grid(True, axis='y')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, bbox_inches='tight', dpi=300)
            print(f"\nSaved comparison plot to: {save_path}")
        plt.show()

        # Print the actual values
        print("\nDetailed Comparison:")
        print(f"Exploration Time: {opt_time/unopt_time*100:.1f}%")
        print(f"Path Length: {opt_length/unopt_length*100:.1f}%")
            
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
        print(f"\nSaved statistics to: {save_path}")

def get_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description='Plot DynaPRM exploration data')
    parser.add_argument('--mode', type=str, choices=['single', 'comparison'],
                      help='Plot mode: single file or comparison between two files')
    return parser.parse_args()

def main():
    '''
    Main function for loading data and plotting comparison plot.
    '''
    args = get_args()
    
    if args.mode == 'single': # Agar 1 file hai toh single plot
        file_path = input("Enter path to CSV file: ")
        if not os.path.exists(file_path):
            print(f"Error: File {file_path} not found!")
            return
            
        plotter = Plotter(file_path)
        plotter.plot_single('single_plot.png')
        plotter.save_stats('stats.csv')
        
    elif args.mode == 'comparison':
        # Prompt for both file paths
        unopt_path = input("Enter path to unoptimized CSV file: ")
        opt_path = input("Enter path to optimized CSV file: ")
        # Checking krro file hai ya nahi
        if not os.path.exists(unopt_path):
            print(f"Error: File {unopt_path} not found!")
            return
        if not os.path.exists(opt_path):
            print(f"Error: File {opt_path} not found!")
            return
            
        # Create plotter and generate comparison
        plotter = Plotter(unopt_path)
        optimized_data = pd.read_csv(opt_path)
        plotter.plot_comparison(optimized_data, 'comparison_plot.png')
        plotter.save_stats('unoptimized_stats.csv')

if __name__ == '__main__':
    main()