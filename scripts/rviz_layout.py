#!/usr/bin/env python3
"""Regenerate a focused RViz dock layout; run with QT_QPA_PLATFORM=offscreen."""
from pathlib import Path
import yaml
from PyQt5.QtWidgets import QApplication, QMainWindow, QDockWidget, QLabel, QToolBar
from PyQt5.QtCore import Qt
app=QApplication([])
w=QMainWindow(); w.setDockNestingEnabled(True); w.resize(1450,850)
w.setCentralWidget(QLabel('Navigation'))
bar=QToolBar('Tools'); bar.setObjectName('Tools');w.addToolBar(bar)
def dock(name,area):
 d=QDockWidget(name);d.setObjectName(name);d.setWidget(QLabel(name));w.addDockWidget(area,d);return d
displays=dock('Displays',Qt.LeftDockWidgetArea)
rgb=dock('RGB downward',Qt.RightDockWidgetArea)
thermal=dock('Thermal and fire detection (290-650 K)',Qt.RightDockWidgetArea)
w.splitDockWidget(rgb,thermal,Qt.Vertical)
time=dock('Time',Qt.BottomDockWidgetArea)
w.show();app.processEvents()
w.resizeDocks([displays,rgb],[330,350],Qt.Horizontal)
w.resizeDocks([rgb,thermal],[350,350],Qt.Vertical)
w.resizeDocks([time],[55],Qt.Vertical)
app.processEvents()
p=Path(__file__).resolve().parents[1]/'tmux/rviz.rviz'
data=yaml.safe_load(p.read_text())
data['Panels']=[panel for panel in data['Panels'] if panel['Name'] in ('Displays','Time')]
data['Window Geometry']['QMainWindow State']=bytes(w.saveState().toHex()).decode()
data['Window Geometry']['Hide Right Dock']=False
for name in ['RGB downward','Thermal and fire detection (290-650 K)']:
 data['Window Geometry'][name]={'collapsed':False}
p.write_text(yaml.safe_dump(data,sort_keys=False))
print('RViz layout written.')
