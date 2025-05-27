from pymavlink import mavutil
import pandas as pd
import numpy as np


def parse_tlog(tlog_path):
    # Open the TLOG file
    mav = mavutil.mavlink_connection(tlog_path)
    
    # List to store extracted data
    gps_data = []
    
    while True:
        msg = mav.recv_match(blocking=False)
        if msg is None:
            break
    
       
        
        # Extract GPS_RAW_INT message
        if msg.get_type() == "GPS_RAW_INT":
            gps_data.append({
                "time_usec": msg.time_usec/1000000,
                "lat": msg.lat / 1e7,  # Convert to degrees
                "lon": msg.lon / 1e7,  # Convert to degrees
                "alt": msg.alt / 1000,  # Convert to meters
                "fix_type": msg.fix_type,
                "eph": msg.eph,
                "epv": msg.epv,
                "vel": msg.vel / 100,  # Convert to m/s
                "cog": msg.cog / 100,  # Convert to degrees
                "satellites_visible": msg.satellites_visible,
                "satellites_visible": msg.satellites_visible,
                "h_acc": msg.h_acc / 10,  # Convert to centimeters
                "v_acc": msg.v_acc / 10,  # Convert to centimeters
                "vel_acc" : msg.vel_acc / 10, # Convert to centim/s
                "hdg_acc" : msg.hdg_acc / 1000 # Convert to degrees
            })
    
    
    # Convert to DataFrame
    gps_raw_ds = pd.DataFrame(gps_data)
    
    # Open the TLOG file
    mav = mavutil.mavlink_connection(tlog_path)
    
    # List to store extracted data
    global_position_data = []
    # List to store yaw data
    yaw_data = []
    
    while True:
        msg = mav.recv_match(blocking=False)
        if msg is None:
            break
    
        
        elif msg.get_type() == "GLOBAL_POSITION_INT":
            global_position_data.append({
                "time_boot_ms": msg.time_boot_ms/1000,  # Time since boot (ms)
                "lat": msg.lat / 1e7,  # Convert to degrees
                "lon": msg.lon / 1e7,  # Convert to degrees
                "alt": msg.alt / 1000,  # Convert to meters (above MSL)
                "relative_alt": msg.relative_alt / 1000,  # Altitude relative to home (m)
                "vx": msg.vx / 100,  # Velocity X (m/s)
                "vy": msg.vy / 100,  # Velocity Y (m/s)
                "vz": msg.vz / 100,  # Velocity Z (m/s)
                "hdg": msg.hdg / 100  # Heading (degrees)
            })     
        elif msg.get_type() == "ATTITUDE":
            yaw_data.append({
                "time_boot_ms": msg.time_boot_ms / 1000,  # Convert to seconds
                "yaw": np.degrees(msg.yaw),  # Convert radians to degrees
                "roll": np.degrees(msg.roll),  # Convert roll from radians to degrees
                "pitch": np.degrees(msg.pitch),  # Convert pitch from radians to degrees
                "yaw": np.degrees(msg.yaw)  # Convert yaw from radians to degrees
            })
        
    
    # Convert to DataFrame
    global_position_ds = pd.DataFrame(global_position_data)
    gps_yaw_data = pd.DataFrame(yaw_data)
    
    # Open the TLOG file
    mav = mavutil.mavlink_connection(tlog_path)
    
    # List to store extracted data
    IMU_data = []
    
    while True:
        msg = mav.recv_match(blocking=False)
        if msg is None:
            break
    
        if msg.get_type() == "SCALED_IMU3":
            imu_data = {
                "timestamp": msg.time_boot_ms / 1000,       # Convert ms to seconds
                "xacc": msg.xacc / 1000.0,                  # Convert from milli-g to g
                "yacc": msg.yacc / 1000.0,
                "zacc": msg.zacc / 1000.0,
                "xgyro": msg.xgyro / 1000.0,                # mrad/s to rad/s
                "ygyro": msg.ygyro / 1000.0,
                "yaw_rate": msg.zgyro / 1000.0              # yaw_rate = zgyro
            }
            IMU_data.append(imu_data)
    
    # Convert to DataFrame
    IMU_ds = pd.DataFrame(IMU_data)
    
    
    #selecting the relevant records
    gps_raw_ds = gps_raw_ds[1:]
    gps_yaw_data = gps_yaw_data[1:]
    IMU_ds = IMU_ds[1:]
    
    #update yaw data
    gps_yaw_data['yaw_updated'] = 0
    gps_yaw_updated = []
    for i in range(len(gps_yaw_data)):
        if i%2 == 0:
            continue
        f_val_yaw = gps_yaw_data['yaw'][i]
        s_val_yaw = gps_yaw_data['yaw'][i+1]
        avg_val_yaw = (f_val_yaw + s_val_yaw)/2
    
        f_val_roll = gps_yaw_data['roll'][i]
        s_val_roll = gps_yaw_data['roll'][i+1]
        avg_val_roll = (f_val_roll + s_val_roll)/2
    
        f_val_pitch = gps_yaw_data['pitch'][i]
        s_val_pitch = gps_yaw_data['pitch'][i+1]
        avg_val_pitch = (f_val_pitch + s_val_pitch)/2
        
        gps_yaw_updated.append({
            'yaw' : avg_val_yaw,
            'roll' : avg_val_roll,
            'pitch' : avg_val_pitch
        })
    gps_yaw_updated = pd.DataFrame(gps_yaw_updated)
    
    #droppping irrelavant columns
    gps_raw_ds = gps_raw_ds.drop("fix_type", axis=1) # min and maximum both values are 3 (accuracy is aprox. 5m)
    gps_raw_ds = gps_raw_ds.drop("time_usec", axis=1)
    gps_raw_ds = gps_raw_ds.drop("lat", axis=1)
    gps_raw_ds = gps_raw_ds.drop("lon", axis=1)
    gps_raw_ds = gps_raw_ds.drop("alt", axis=1)
    gps_raw_ds = gps_raw_ds.drop("hdg_acc", axis=1)
    IMU_ds = IMU_ds.drop("timestamp", axis=1)
    
    #performing the positional shift fro global_position_ds and gps_yaw_updated
    pos_shift = 0
    if pos_shift == 0:
        global_position_ds.index = global_position_ds.index + 1 #uncomment when doing first
        gps_yaw_updated.index = gps_yaw_updated.index + 1 #uncomment when doing first
        pos_shift = 1
    else:
        pass
    
    #concatenation of datasets
    ds_final = pd.concat([global_position_ds, gps_raw_ds, IMU_ds, gps_yaw_updated], axis=1)
    
    ds_final = ds_final.reset_index(drop=True)

    from math import cos, radians
    
    def xy_distance(lat1, lon1, lat2, lon2):
        lat_avg = radians((lat1 + lat2) / 2)  # Average latitude in radians
    
        # Y-axis distance (latitude)
        dy = (lat2 - lat1) * 111139  # More precise value
    
        # X-axis distance (longitude)
        dx = (lon2 - lon1) * (111139 * cos(lat_avg))  # Adjust for longitude
    
        return dx, dy  # Distances in meters
    
    ds_final['latitude_dif(cm)'] = 0
    ds_final['longitude_dif(cm)'] = 0
    #print(ds_final.head())
    def diff_algo1(dataset):
        ln = len(dataset)
        for i in range(ln):
            lon_val = dataset['lon'][i]
            lat_val = dataset['lat'][i]
            if i == 0:
                prev_lon_val = lon_val
                prev_lat_val = lat_val
                continue
            dx, dy = xy_distance(prev_lat_val, prev_lon_val, lat_val, lon_val)
            
            dataset['latitude_dif(cm)'][i] = dy.round(4)*100
            dataset['longitude_dif(cm)'][i] = dx.round(4)*100
            prev_lon_val = lon_val
            prev_lat_val = lat_val
    
    diff_algo1(ds_final)  
    

    # Convert timestamps into seconds (from milliseconds)
    ds_final['timestamp_seconds'] = (ds_final['time_boot_ms'] - ds_final['time_boot_ms'].iloc[0])
    
    
    ds_final['timestamp_sec_diff'] = 0
    ds_final['latitude_diff(deg)'] = 0
    ds_final['longitude_diff(deg)'] = 0
    
    def diff_algo_time1(dataset):
        ln = len(dataset)
        for i in range(ln):
            time_val = dataset['timestamp_seconds'][i]
            if i == 0:
                prev_time_val = time_val
                continue
            dif = time_val - prev_time_val
            dataset['timestamp_sec_diff'][i] = dif
            prev_time_val = time_val
    
    
    def diff_algo_lat1(dataset):
        ln = len(dataset)
        for i in range(ln):
            lt_val = dataset['lat'][i]
            if i == 0:
                prev_lt_val = lt_val
                continue
            dif = lt_val - prev_lt_val
            dataset['latitude_diff(deg)'][i] = dif
            prev_lt_val = lt_val
    
    
    def diff_algo_lon1(dataset):
        ln = len(dataset)
        for i in range(ln):
            lt_val = dataset['lon'][i]
            if i == 0:
                prev_lt_val = lt_val
                continue
            dif = lt_val - prev_lt_val
            dataset['longitude_diff(deg)'][i] = dif
            prev_lt_val = lt_val
    
    diff_algo_lon1(ds_final)
    diff_algo_time1(ds_final)
    diff_algo_lat1(ds_final)
    
    
    
    #scaling ton 100ms
    new_data = {'Latitude':[], 'Longitude':[], 'Altitude':[], 'Timestamp_seconds':[], 'rel_alt_val':[], 'vx':[], 'vy':[], 'vz':[], 'hdg':[], 'cog':[], 'satellites':[], 'h_acc':[], 'v_acc':[], 'vel_acc':[], 'xacc':[], 'yacc':[], 'zacc':[], 'yaw':[], 'xgyro':[], 'ygyro':[], 'yaw_rate':[], 'roll':[], 'pitch':[], 'eph':[]}
    def new_df(dat):
        
        for i in range(len(dat)):
            if i == 0:
                continue
            tim = dat['timestamp_sec_diff'][i]
            div = np.ceil(tim / 0.1).astype(int)
            #print(div)
            #latitude
            lat_val_diff = dat['latitude_diff(deg)'][i]/div
            lat_val = dat['lat'][i-1]
            #longitude
            lon_val_diff = dat['longitude_diff(deg)'][i]/div
            lon_val = dat['lon'][i-1]
            #altitude
            alt_val_diff = (dat['alt'][i]-dat['alt'][i-1])/div
            alt_val = dat['alt'][i-1]
            #rel_altitude
            rel_alt_val_diff = (dat['relative_alt'][i]-dat['relative_alt'][i-1])/div
            rel_alt_val = dat['relative_alt'][i-1]
    
            #vx
            vx_val_diff = (dat['vx'][i]-dat['vx'][i-1])/div
            vx_val = dat['vx'][i-1]
            #vy
            vy_val_diff = (dat['vy'][i]-dat['vy'][i-1])/div
            vy_val = dat['vy'][i-1]
            #vz
            
            vz_val_diff = (dat['vz'][i]-dat['vz'][i-1])/div
            vz_val = dat['vz'][i-1]
    
            #hdg
            hdg_val_diff = (dat['hdg'][i]-dat['hdg'][i-1])/div
            hdg_val = dat['hdg'][i-1]
    
            #cog
            cog_val_diff = (dat['cog'][i]-dat['cog'][i-1])/div
            cog_val = dat['cog'][i-1]
    
            #satellite
            satellite_div = (dat['satellites_visible'][i]-dat['satellites_visible'][i-1])/div
            satellite_prev_val = dat['satellites_visible'][i-1]
            #h_acc
            h_acc_div = (dat['h_acc'][i]-dat['h_acc'][i-1])/div
            h_acc_prev_val = dat['h_acc'][i-1]
    
            #v_acc
            v_acc_div = (dat['v_acc'][i]-dat['v_acc'][i-1])/div
            v_acc_prev_val = dat['v_acc'][i-1]
    
            #vel_acc
            vel_acc_div = (dat['vel_acc'][i]-dat['vel_acc'][i-1])/div
            vel_acc_prev_val = dat['vel_acc'][i-1]
    
            #x_acc
            x_acc_div = (dat['xacc'][i]-dat['xacc'][i-1])/div
            x_acc_prev_val = dat['xacc'][i-1]
            #y_acc
            y_acc_div = (dat['yacc'][i]-dat['yacc'][i-1])/div
            y_acc_prev_val = dat['yacc'][i-1]
            #z_acc
            z_acc_div = (dat['zacc'][i]-dat['zacc'][i-1])/div
            z_acc_prev_val = dat['zacc'][i-1]
    
            #yaw
            yaw_div = (dat['yaw'][i]-dat['yaw'][i-1])/div
            yaw_prev_val = dat['yaw'][i-1]
    
            #yaw rate
            yaw_rate_div = (dat['yaw_rate'][i]-dat['yaw_rate'][i-1])/div
            yaw_rate_prev_val = dat['yaw_rate'][i-1]
    
            #x gyro
            xgyro_div = (dat['xgyro'][i]-dat['xgyro'][i-1])/div
            xgyro_prev_val = dat['xgyro'][i-1]
    
            #y gyro
            ygyro_div = (dat['ygyro'][i]-dat['ygyro'][i-1])/div
            ygyro_prev_val = dat['ygyro'][i-1]
    
            #roll
            roll_div = (dat['roll'][i]-dat['roll'][i-1])/div
            roll_prev_val = dat['roll'][i-1]
    
            #pitch
            pitch_div = (dat['pitch'][i]-dat['pitch'][i-1])/div
            pitch_prev_val = dat['pitch'][i-1]
    
            #eph
            eph_div = (dat['eph'][i]-dat['eph'][i-1])/div
            eph_prev_val = dat['eph'][i-1]
    
            
            tim_val = dat['timestamp_seconds'][i-1]
            
            
            for j in range(div):
                new_data['Timestamp_seconds'].append(tim_val+j*0.1)
                new_data['Latitude'].append((lat_val+lat_val_diff*(j)).round(7))
                new_data['Longitude'].append((lon_val+lon_val_diff*(j)).round(7))
                new_data['Altitude'].append((alt_val+alt_val_diff*(j)).round(7))
                new_data['rel_alt_val'].append((rel_alt_val+rel_alt_val_diff*(j)).round(7))
                new_data['vx'].append((vx_val+vx_val_diff*(j)).round(7))
                new_data['vy'].append((vy_val+vy_val_diff*(j)).round(7))
                new_data['vz'].append((vz_val+vz_val_diff*(j)).round(7))
                new_data['hdg'].append((hdg_val+hdg_val_diff*(j)).round(7))
                new_data['cog'].append((cog_val+cog_val_diff*(j)).round(7))
                new_data['satellites'].append((satellite_prev_val+j*satellite_div).round(7))
                new_data['h_acc'].append((h_acc_prev_val+j*h_acc_div).round(7))
                new_data['v_acc'].append((v_acc_prev_val+j*v_acc_div).round(7))
                new_data['vel_acc'].append((vel_acc_prev_val+j*vel_acc_div).round(7))
                new_data['xacc'].append((x_acc_prev_val+j*x_acc_div).round(7))
                new_data['yacc'].append((y_acc_prev_val+j*y_acc_div).round(7))
                new_data['zacc'].append((z_acc_prev_val+j*z_acc_div).round(7))
                new_data['yaw'].append((yaw_prev_val+j*yaw_div).round(7))
                new_data['yaw_rate'].append((yaw_rate_prev_val+j*yaw_rate_div).round(7))
                new_data['xgyro'].append((xgyro_prev_val+j*xgyro_div).round(7))
                new_data['ygyro'].append((ygyro_prev_val+j*ygyro_div).round(7))
                new_data['roll'].append((roll_prev_val+j*roll_div).round(7))
                new_data['pitch'].append((pitch_prev_val+j*pitch_div).round(7))
                new_data['eph'].append((eph_prev_val+j*eph_div).round(7))
    
    new_df(ds_final)
    scaled_df = pd.DataFrame(new_data)
    
    scaled_df['latitude_dif(cm)'] = 0
    scaled_df['longitude_dif(cm)'] = 0
    scaled_df['Timestamp_sec_diff'] = 0
    scaled_df['latitude_diff(deg)'] = 0
    scaled_df['longitude_diff(deg)'] = 0
    
    
    from math import cos, radians
    
    def xy_distance(lat1, lon1, lat2, lon2):
        lat_avg = radians((lat1 + lat2) / 2)  # Average latitude in radians
    
        # Y-axis distance (latitude)
        dy = (lat2 - lat1) * 111139  # More precise value
    
        # X-axis distance (longitude)
        dx = (lon2 - lon1) * (111139 * cos(lat_avg))  # Adjust for longitude
    
        return dx, dy  # Distances in meters
    
    def diff_algo2(dataset):
        ln = len(dataset)
        for i in range(ln):
            lon_val = dataset['Longitude'][i]
            lat_val = dataset['Latitude'][i]
            if i == 0:
                prev_lon_val = lon_val
                prev_lat_val = lat_val
                continue
            dx, dy = xy_distance(prev_lat_val, prev_lon_val, lat_val, lon_val)
            
            dataset['latitude_dif(cm)'][i] = dy.round(4)*100
            dataset['longitude_dif(cm)'][i] = dx.round(4)*100
            prev_lon_val = lon_val
            prev_lat_val = lat_val
         
    
    
    def diff_algo_time2(dataset):
        ln = len(dataset)
        for i in range(ln):
            time_val = dataset['Timestamp_seconds'][i]
            if i == 0:
                prev_time_val = time_val
                continue
            dif = time_val - prev_time_val
            dataset['Timestamp_sec_diff'][i] = dif
            prev_time_val = time_val
    
    
    
    def diff_algo_lat2(dataset):
        ln = len(dataset)
        for i in range(ln):
            lt_val = dataset['Latitude'][i]
            if i == 0:
                prev_lt_val = lt_val
                continue
            dif = lt_val - prev_lt_val
            dataset['latitude_diff(deg)'][i] = dif
            prev_lt_val = lt_val
    
    
    def diff_algo_lon2(dataset):
        ln = len(dataset)
        for i in range(ln):
            lt_val = dataset['Longitude'][i]
            if i == 0:
                prev_lt_val = lt_val
                continue
            dif = lt_val - prev_lt_val
            dataset['longitude_diff(deg)'][i] = dif
            prev_lt_val = lt_val
    
    
    diff_algo_lon2(scaled_df)
    diff_algo2(scaled_df)
    diff_algo_time2(scaled_df)
    diff_algo_lat2(scaled_df)
    
    scaled_df.drop(['latitude_diff(deg)'], axis = 1, inplace = True)
    scaled_df.drop(['longitude_diff(deg)'], axis = 1, inplace = True)
    
    
    scaled_df['latitude_diff_diff(cm)'] = 0
    scaled_df['longitude_diff_diff(cm)'] = 0
    
    def diff_of_diff_algo(dataset):
        ln = len(dataset)
        for i in range(ln):
            lon_val = dataset['longitude_dif(cm)'][i]
            lat_val = dataset['latitude_dif(cm)'][i]
            if i == 0:
                prev_lon_val = lon_val
                prev_lat_val = lat_val
                continue
            dx = lon_val - prev_lon_val
            dy = lat_val - prev_lat_val
            
            dataset['latitude_diff_diff(cm)'][i] = dy
            dataset['longitude_diff_diff(cm)'][i] = dx
            prev_lon_val = lon_val
            prev_lat_val = lat_val
    
    diff_of_diff_algo(scaled_df)
    return scaled_df
