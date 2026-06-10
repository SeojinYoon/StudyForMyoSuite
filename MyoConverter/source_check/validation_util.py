
# Common Libraries
import matplotlib.pylab as plt
import numpy as np

# Functions
def plot_moment_arm(muscle_name, 
                    joint_name,
                    joint_range_osim, 
                    joint_range_mjc, 
                    ma_mat_osim, 
                    ma_mat_mjc,
                    n_evals,
                    n_joint,
                    save_img_path,
                    is_aggregation = False):
    # Figure
    f = plt.figure(figsize=(10, 8))
    ax1 = f.add_subplot(1, 2, 1)
    ax2 = f.add_subplot(1, 2, 2)

    # X
    x_osim = np.linspace(joint_range_osim[0], joint_range_osim[1], n_evals, endpoint=True)
    x_mjc = np.linspace(joint_range_mjc[0], joint_range_mjc[1], n_evals, endpoint=True)

    # Find the minimal and maximum values in mjc and osim ma data, to set the axis equal
    all_data = np.concatenate([ma_mat_osim, -ma_mat_mjc]) # osim and mjc model have oppsite signs in moment arms
    max_ma = np.max(all_data)
    min_ma = np.min(all_data)

    # Plot data
    n_joint_config = n_evals**(n_joint-1)

    # colormap
    cmap = plt.get_cmap('jet')
    line_color = [cmap(c / n_joint_config) for c in range(n_joint_config)]

    if is_aggregation == False:
        for c in range(n_joint_config):
            ax1.plot(x_osim, ma_mat_osim[:, c]*100, color=line_color[c], lw=1.5, alpha=0.7, marker = "s")
        
        for c in range(n_joint_config):
            ax2.plot(x_mjc, -ma_mat_mjc[:, c]*100, color=line_color[c], lw=1.5, alpha=0.7, marker = "s")
    else:
        osim_ma_mean = np.mean(ma_mat_osim, axis=1) * 100
        osim_ma_std = np.std(ma_mat_osim, axis=1) * 100
        ax1.fill_between(x_osim, osim_ma_mean - osim_ma_std, osim_ma_mean + osim_ma_std, alpha=0.2)
        ax1.plot(x_osim, osim_ma_mean, lw=2, marker="s", markersize=4)

        mjc_ma_mean = np.mean(-ma_mat_mjc, axis=1) * 100
        mjc_ma_std = np.std(-ma_mat_mjc, axis=1) * 100
        ax2.fill_between(x_mjc, mjc_ma_mean - mjc_ma_std, mjc_ma_mean + mjc_ma_std, alpha=0.2)
        ax2.plot(x_mjc, mjc_ma_mean, lw=1.5, alpha=0.7, marker = "s")
        
    # Axis limit
    ax1.set_ylim([min_ma*100, max_ma*100])
    ax2.set_ylim([min_ma*100, max_ma*100])

    # Label
    ax1.set_xlabel(joint_name + " (rad)")
    ax1.set_ylabel("moment arms (cm)")

    ax2.set_xlabel(joint_name + " (rad)")
    ax2.set_ylabel("moment arms (cm)")

    # Title
    ax1.set_title("OSIM")
    ax2.set_title("MJC")
    plt.suptitle(f"{joint_name} - {muscle_name}")

    # Save figure
    if save_img_path is not None:
        f.savefig(save_img_path, format = "png")
        plt.close(f)
        