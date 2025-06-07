import streamlit as st
import pandas as pd
import matplotlib.pyplot as plt
import tempfile
from tlog_parser import parse_tlog
from ukf_module import run_ukf
from math import cos, radians

st.set_page_config(page_title="TLOG GPS Analyzer", layout="wide")
st.title("📍 GPS TLOG Analyzer")

# Upload Section
uploaded_file = st.file_uploader("Upload a `.tlog` file", type=["tlog"])

if uploaded_file is not None:
    with tempfile.NamedTemporaryFile(delete=False, suffix=".tlog") as tmp_file:
        tmp_file.write(uploaded_file.read())
        tmp_path = tmp_file.name

    with st.spinner("Parsing and resampling TLOG..."):
        df = parse_tlog(tmp_path)

    st.success("✅ File parsed successfully!")
    st.write(f"Parsed {len(df)} samples at 10 Hz")

    # Compute Latitude/Longitude Diffs
    def xy_distance(lat1, lon1, lat2, lon2):
        lat_avg = radians((lat1 + lat2) / 2)
        dy = (lat2 - lat1) * 111139
        dx = (lon2 - lon1) * (111139 * cos(lat_avg))
        return dx, dy

    df['latitude_dif(cm)'] = 0.0
    df['longitude_dif(cm)'] = 0.0
    for i in range(1, len(df)):
        dx, dy = xy_distance(df.loc[i-1, 'Latitude'], df.loc[i-1, 'Longitude'], df.loc[i, 'Latitude'], df.loc[i, 'Longitude'])
        df.loc[i, 'latitude_dif(cm)'] = round(dy * 100, 4)
        df.loc[i, 'longitude_dif(cm)'] = round(dx * 100, 4)
    
    # Download button for raw + difference DataFrame
    st.subheader("📥 Download Parsed + Difference Data")
    csv_raw = df.to_csv(index=False).encode('utf-8')
    st.download_button(
        label="Download Preprocessed DataFrame (with lat/lon diff)",
        data=csv_raw,
        file_name="preprocessed_latlon_diff.csv",
        mime='text/csv'
    )

    # Module 2: Plot Signals
    st.header("📈 Module 2: Visualize Raw & Other Signals")
    st.sidebar.header("🛠️ Plot Settings")

    x_axis = st.sidebar.radio("X-axis", ['Timestamp_seconds', 'Index'])
    if 'Index' not in df.columns:
        df['Index'] = df.index

    available_signals = [
        'Latitude', 'Longitude', 'xacc', 'yacc', 'zacc',
        'roll', 'pitch', 'yaw', 'yaw_rate',
        'latitude_dif(cm)', 'longitude_dif(cm)'
    ]

    plot_options = st.sidebar.multiselect("Select signals to plot:", available_signals, default=['Latitude', 'Longitude'])

    selected_colors = {}
    if plot_options:
        st.sidebar.markdown("🎨 **Pick plot colors**:")
        for signal in plot_options:
            selected_colors[signal] = st.sidebar.color_picker(f"{signal} color", "#1f77b4")

    if plot_options:
        x_vals = df['Index'] if x_axis == 'Index' else df['Timestamp_seconds']
        fig, axs = plt.subplots(len(plot_options), 1, figsize=(10, 2.5 * len(plot_options)), sharex=True)
        if len(plot_options) == 1:
            axs = [axs]
        for ax, signal in zip(axs, plot_options):
            ax.plot(x_vals, df[signal], label=signal, color=selected_colors[signal])
            ax.set_ylabel(signal)
            ax.legend()
        axs[-1].set_xlabel(x_axis.replace("_", " ").title())
        plt.tight_layout()
        st.pyplot(fig)
    else:
        st.info("☝️ Select at least one variable to plot from the sidebar.")

    # Module 3: UKF Filtering
    st.header("📉 Module 3: UKF Filtering")

    scenario = st.radio("Select UKF Scenario:", ["Scenario I (With Attitude)", "Scenario II (IMU Only)"])
    if scenario == "Scenario I (With Attitude)":
    	include_attitude = True
    else:
    	include_attitude = False


    if st.button("Run UKF Filter"):
        st.info("Running UKF filtering...")
        results_df, fig = run_ukf(df, include_attitude=include_attitude, plot=True)
        st.pyplot(fig)
        st.success("UKF completed and plotted.")

else:
    st.warning("📤 Please upload a `.tlog` file to begin.")
