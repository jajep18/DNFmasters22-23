import plotly as py
import plotly.graph_objs as go
import numpy as np
import pandas as pd
import plotly.express as px



def plot_succeeded_pos():
    df = pd.read_csv('control_datalog.csv')
    print("Number of points: ", len(df))
    df = df[df['success'] == 1]

    # Remove scale of 1000
    df['x'] = df['x'] / 1000
    df['y'] = df['y'] / 1000
    df['z'] = df['z'] / 1000



    # Create custom colorscale going from green to red
    colorscale = [[0, 'red'], [1, 'green']]
    df[['size']] = np.abs(df[['success']] + 0.5)
    

    fig = px.scatter_3d(df, x="x", y="y", z="z", color="success",
                        color_continuous_scale=colorscale, range_color=[0, 1], size= 'size', opacity=1,
                        labels={'x':'x (m)', 'y':'y (m)', 'z':'z (m)'}) 
    # remove the colorbar
    fig.update_layout(coloraxis_showscale=False,
                      # Add title
                        title={
                            'text': "3D Reachability map",
                            'y':0.8,
                            'x':0.5,
                            'xanchor': 'center',
                            'yanchor': 'top'},
                        font=dict(
                            family="Times New Roman",
                            size=25,
                            color="black",
                        )
                    )
            
    # fig.show()
    
    # Count number of z points in each coordinate
    df['z_count'] = df.groupby(['x', 'y'])['z'].transform('count')
    # Remove counts of 1
    # df = df[df['z_count'] > 1]

    df['size']= df['z_count']*10
    
    # Create a 2d scatter plot of the x, y coordinates with the z_count as the color
    fig = px.scatter(df, x="x", y="y", color="z_count", size=("size"), color_continuous_scale='Aggrnyl_r', 
                     title="2D Reachability map", labels={'x':'x (m)', 'y':'y (m)', 'z_count':'Z Points'})
    fig.update_layout(
                    # Set font size to 35
                    font=dict(
                        family="Times New Roman",
                        size=40,
                        color="black",
                    ),
                    )
    # Plot a square in the plot
    # fig.add_shape(type="rect",
    #             x0=-0.35, y0=-0.1, x1=0.45, y1=-0.35,
    #             line=dict(
    #                 color="RoyalBlue",
    #                 width=2,
    #                 dash="dashdot",
    #             ),
    #             )


    # # Change the size of the points
    # fig.update_traces(marker=dict(size=12,
    #                             line=dict(width=2),
    #                 ))
    

    # fig.show()


    # df = pd.read_csv('control_datalog.csv')
    # df = df[df['success'] == 1]


    # print("Number of reachable points: ", len(df))
    

    # Keep only the rows where the control succeeded
    df = df[df['success'] == 1]
    print("Number of reachable points: ", len(df))
    # print("Max x", df['x'].max())
    # print("Min x", df['x'].min())
    # print("Max y", df['y'].max())
    # print("Min y", df['y'].min())
    # print("Max z", df['z'].max())
    # print("Min z", df['z'].min())

# <size> 0.5 0.2 0.01 </size>
    # Calculate the coverage presentage of the reachable points in a square of 0.5x0.2x0.01 with center in (0, -0.22, -0.1)
    length = 0.5
    width = 0.2
    x_center = 0
    y_center = -0.175
    

    df = df[df['x'] < x_center + length/2]
    df = df[df['x'] > x_center - length/2]
    df = df[df['y'] < y_center + width/2]
    df = df[df['y'] > y_center - width/2]

    # Print max and min values of the square calculated
    print("Max x", x_center + length/2)
    print("Min x", x_center - length/2)
    print("Max y", y_center + width/2)
    print("Min y", y_center - width/2)

    # -0.075620
    

    print("Number of reachable points in square: ", len(df))



    print("Number of reachable points in square: ", len(df))





    # Plot the reachable points in the square in 2D
    fig = px.scatter(df, x="x", y="y", color="z_count", size=("size"), color_continuous_scale='Aggrnyl_r', 
                     title="2D Reachability on the worksurface", labels={'x':'x (m)', 'y':'y (m)', 'z_count':'Z Points'})
    fig.update_layout(
                    # Set font size to 35
                    font=dict(
                        family="Times New Roman",
                        size=40,
                        color="black",
                    ),
                    )
    
    # Plot a square in the plot 
    fig.add_shape(type="rect",
                x0=x_center-length/2, y0=y_center-width/2, x1=x_center+length/2, y1=y_center+width/2,
                line=dict(
                    color="RoyalBlue",
                    width=2,
                    dash="dashdot",
                ),
                )
                  



    
    fig.show()
   
    


    










def main():
    plot_succeeded_pos()

if __name__ == '__main__':
    
    main()

