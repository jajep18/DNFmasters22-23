import plotly as py
import plotly.graph_objs as go
import numpy as np
import pandas as pd
import plotly.express as px



def triangulation_sorted():
    # Read the data from the csv file
    # df = pd.read_csv('triangulation_datalog-all.csv')
    df = pd.read_csv('triangulation_datalog_v2.csv')


    # Create the figure
    # fig = go.Figure()

    # Calculate the distance between x and y and the ground truth
    df['dist_x'] = df['x'] - df['gt_x']
    df['dist_y'] = df['y'] - df['gt_y']
    df['dist'] = np.sqrt(df['dist_x']**2 + df['dist_y']**2)
    # Create custom colorscale going from green to red
    colorscale = [[0, 'green'], [0.5, 'yellow'], [1, 'red']]


    # Remove dist larger than 0.035
    df = df[df['dist'] < 0.035]
    

    # Add a size column to the dataframe
    df['size'] = 1
    max_dist = df['dist'].max()
    min_dist = df['dist'].min()
    dist_range = [min_dist, max_dist]

    
    

    fig = go.Figure()
    # Create a scatter plot of the triangulated circles with the ground truth
    fig = px.scatter(df, x="x", y="y", color="dist", size="size",
                     color_continuous_scale=colorscale, range_color=dist_range, 
                     labels={'x':'x (m)', 'y':'y (m)'}, title="Triangulation error")
    fig.update_traces(marker={'size': 50})
    # Add the scatter plot to the legend
    fig.add_trace(go.Scatter(
        x=[None],
        y=[None],
        mode='markers',
        marker=dict(size=10, color='green'),
        name='Triangulated coordinates'
    ))

    # Add line between ground truth and triangulated data in the scatter plot
    fig.add_trace(go.Scatter(
        x=[df['gt_x'], df['x']],
        y=[df['gt_y'], df['y']],
        mode='lines',
        line=dict(color='black', width=2),
        name='Distance between ground truth and triangulated coordinates'
    ))




    # Add title 
    fig.update_layout(
        title={
            'text': "Triangulation error",
            'y':0.99,
            'x':0.2,
            'xanchor': 'center',
            'yanchor': 'top'},
        xaxis_title="x (m)",
        yaxis_title="y (m)",
        font=dict(
            family="Times New Roman",
            size=40,
            color="black",

        )
    )

                  
    # Add ground truth and triangulated data to the legend
    fig.update_layout(
        legend_title="Legend:",
        legend=dict(
            yanchor="top",
            y=0.95,
            xanchor="right",
            x=0.01,
            title_font_family="Times New Roman",
            # Add ground truth and triangulated data to the legend
            itemsizing='constant',

        ),
        font=dict(
            family="Times New Roman",
            size=35,
            color="Black"
        )
    )

    # Change the legend orientation to horizontal and move it to the top right
    fig.update_layout(legend=dict(
    orientation="h",
    yanchor="bottom",
    y=1.02,
    xanchor="right",
    x=1
    ))
    # Change the colorbar title
    fig.update_coloraxes(colorbar_title="Error (m)")


    # Add the ground truth
    fig.add_trace(go.Scatter(
        x=df['gt_x'],
        y=df['gt_y'],
        mode='markers',
        name='Ground truth',
        marker=dict(
            # color=df['dist'],
            color='black',
            colorscale='Viridis',
            size=20,
            symbol='x'
        ),
        # Add distance to the hover text
        # hovertext=df['dist']
    ))


    fig.show()

    # Calculate the mean error
    mean_error = df['dist'].mean()
    # Show the mean error
    print('Mean error: ' + str(mean_error))
    # Calculate the median error
    median_error = df['dist'].median()
    # Show the median error
    print('Median error: ' + str(median_error))
    # Calculate the standard deviation
    std_dev = df['dist'].std()
    # Show the standard deviation
    print('Standard deviation: ' + str(std_dev))
    # Calculate the max error
    max_error = df['dist'].max()
    # Show the max error
    print('Max error: ' + str(max_error))
    # Calculate the min error
    min_error = df['dist'].min()
    # Show the min error
    print('Min error: ' + str(min_error))


    # Create a histogram of the error
    fig_hist = px.histogram(df, x="dist", nbins=50, labels={'dist':'Error (m)'}, title="Error histogram")
    fig_hist.update_layout(
        title={
            'text': "Error histogram",
            'y':0.95,
            'x':0.5,
            'xanchor': 'center',
            'yanchor': 'top'},
        xaxis_title="Error (m)",
        yaxis_title="Count",
        font=dict(
            family="Times New Roman",
            size=35,
            color="black",

        )
    )
    # Count number of points below 0.005 m
    count = df[df['dist'] <= 0.005]['dist'].count()
    print('Number of points below 0.005 m: ' + str(count))
    fig_hist.show()
    

        
def triangulation_unsorted():

    df = pd.read_csv('triangulation_datalog-noise007.csv')
    # df = pd.read_csv('triangulation_datalog-all.csv')
    linewidth = 4

    # Create two line plots for the x and y coordinates and the ground truth
    fig_x = go.Figure()
    fig_y = go.Figure()

    # Add the ground truth
    fig_x.add_trace(go.Line(x=df['gt_x'].index, y=df['gt_x'], name='Ground truth x'))
    fig_y.add_trace(go.Line(x=df['gt_y'].index, y=df['gt_y'], name='Ground truth y'))

    fig_x.update_traces(line={'width': linewidth, 'color':'blue'})
    fig_y.update_traces(line={'width': linewidth, 'color':'blue'})

    # Add the triangulated data
    fig_x.add_trace(go.Line(x=df['x'].index, y=df['x'], name='Triangulated x'))
    fig_y.add_trace(go.Line(x=df['y'].index, y=df['y'], name='Triangulated y'))

    
    fig_y.update_traces(line=dict(width=4))
    fig_x.update_traces(line=dict(width=4))

    

    # Add title
    fig_x.update_layout(
        title={
            'text': "Triangulation delay",
            'y':0.95,
            'x':0.5,
            'xanchor': 'center',
            'yanchor': 'top'},
        xaxis_title="Data points",
        yaxis_title="X (m)",
        font=dict(
            family="Times New Roman",
            size=35,
            color="black",

        )
    )
    fig_y.update_layout(
        title={
            'text': "Triangulation delay",
            'y':0.95,
            'x':0.5,
            'xanchor': 'center',
            'yanchor': 'top'},
        xaxis_title="Data points",
        yaxis_title="Y (m)",
        font=dict(
            family="Times New Roman",
            size=35,
            color="black",

        )
    )




    # Show the plot
    fig_x.show()
    fig_y.show()




def main():
    triangulation_sorted()
    triangulation_unsorted()

if __name__ == '__main__':
    main()

