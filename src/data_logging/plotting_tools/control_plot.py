import plotly as py
import plotly.graph_objs as go
import numpy as np
import pandas as pd
import plotly.express as px



def plot_succeeded_pos():
    df = pd.read_csv('control_datalog.csv')
    df = df[df['success'] == 1]

    # Remove scale of 1000
    df['x'] = df['x'] / 1000
    df['y'] = df['y'] / 1000
    df['z'] = df['z'] / 1000



    # Create custom colorscale going from green to red
    colorscale = [[0, 'red'], [1, 'green']]

    fig = px.scatter_3d(df, x="x", y="y", z="z", color="success",
                        color_continuous_scale=colorscale, range_color=[0, 1],
                        labels={'x':'x (m)', 'y':'y (m)', 'z':'z (m)'}, title="Reachability map") 
    

    fig.show()

    # Keep only the rows where the control succeeded
    df = df[df['success'] == 1]
    print("Max x", df['x'].max())
    print("Min x", df['x'].min())
    print("Max y", df['y'].max())
    print("Min y", df['y'].min())
    print("Max z", df['z'].max())
    print("Min z", df['z'].min())









def main():
    plot_succeeded_pos()

if __name__ == '__main__':
    
    main()

