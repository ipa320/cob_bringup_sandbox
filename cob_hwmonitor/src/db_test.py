#!/usr/bin/env python

import sqlite3

db = sqlite3.connect('/home/uhr-eh/git/care-o-bot/cob_apps/cob_hwmonitor/db/hwmonitor.db')
cursor = db.cursor()
cursor.execute('select * from hwmonitor')

for row in cursor:
    print row