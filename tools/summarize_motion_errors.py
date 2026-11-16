"""Exact direct-error metrics with on-disk quantiles, never fits lag/offset."""
import argparse
import csv
import json
import math
import sqlite3
import tempfile

def summarize(path,error_column='error_deg',speed_column=None):
    with tempfile.TemporaryDirectory() as directory:
        db=sqlite3.connect(directory+'/errors.db')
        try:
            db.execute('create table errors(value real, signed real, speed text)')
            count=missing=0;square=absolute=total=0.
            speed_rows={}
            with open(path,encoding='utf-8-sig',newline='') as f:
                reader=csv.DictReader(f)
                if error_column not in (reader.fieldnames or []):raise ValueError('CSV thiếu cột sai số đã đối chiếu')
                if speed_column and speed_column not in reader.fieldnames:raise ValueError('CSV thiếu cột vận tốc đã chọn')
                for row in reader:
                    speed='unknown'
                    if speed_column and row[speed_column]!='':
                        velocity=abs(float(row[speed_column]))
                        if not math.isfinite(velocity):raise ValueError('Vận tốc không hữu hạn')
                        speed=next(label for bound,label in [(30,'0_to_30'),(90,'30_to_90'),(180,'90_to_180'),(math.inf,'above_180')] if velocity<=bound)
                    speed_rows[speed]=speed_rows.get(speed,0)+1
                    if row[error_column]=='':missing+=1;continue
                    value=float(row[error_column])
                    if not math.isfinite(value):raise ValueError('Sai số không hữu hạn')
                    count+=1;square+=value*value;absolute+=abs(value);total+=value
                    db.execute('insert into errors values (?,?,?)',(abs(value),value,speed))
            p95=None
            if count:
                rank=(count-1)*.95;i=math.floor(rank)
                values=[v[0] for v in db.execute('select value from errors order by value limit 2 offset ?',(i,))]
                p95=values[0]+(values[-1]-values[0])*(rank-i)
            bins={}
            if speed_column:
                for speed,rows in speed_rows.items():
                    n,mean_square,mean_abs,bias=db.execute('select count(*),avg(signed*signed),avg(value),avg(signed) from errors where speed=?',(speed,)).fetchone()
                    quantile=None
                    if n:
                        rank=(n-1)*.95;i=math.floor(rank)
                        vals=[v[0] for v in db.execute('select value from errors where speed=? order by value limit 2 offset ?',(speed,i))]
                        quantile=vals[0]+(vals[-1]-vals[0])*(rank-i)
                    bins[speed]={'count':n,'rmse_deg':math.sqrt(mean_square) if n else None,'mae_deg':mean_abs,'bias_deg':bias,'p95_abs_deg':quantile,'valid_fraction_of_input_rows':n/rows}
            return {'count':count,'missing_reference_rows':missing,'rmse_deg':math.sqrt(square/count) if count else None,
                    'mae_deg':absolute/count if count else None,'bias_deg':total/count if count else None,
                    'p95_abs_deg':p95,'valid_fraction_of_input_rows':count/(count+missing) if count+missing else None,
                    'processing':'direct provided error column; no lag/offset fitting',
                    'coverage_note':'fraction of CSV rows, not coverage of unreceived samples',
                    'speed_bins':bins,'speed_source':speed_column,'speed_units':'deg/s' if speed_column else None}
        finally:db.close()
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('csv');p.add_argument('--error-column',default='error_deg');p.add_argument('--speed-column');a=p.parse_args()
    print(json.dumps(summarize(a.csv,a.error_column,a.speed_column),indent=2))
