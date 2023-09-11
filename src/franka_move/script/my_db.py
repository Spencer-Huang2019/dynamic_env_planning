import pymysql
from datetime import datetime


class DbManagement():
    def __init__(self):
        self.conn = pymysql.connect(
            host="192.168.48.1",
            port=3306,
            database="robot_platform",
            charset="utf8",
            user="spencer",
            passwd="HHHsss111"
        )

    def insert_data(self, sql):
        try:
            with self.conn.cursor() as cursor:
                date_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                sql = sql + "str_to_date('%s', '%%Y - %%m - %%d %%H: %%i: %%S'));" % (date_time)
                print(sql)
                cursor.execute(sql)
                id = cursor.lastrowid
                self.conn.commit()
                return True, id
        except Exception as e:
            self.conn.rollback()
            return False, None

    def search_data(self, sql):
        try:
            with self.conn.cursor() as cursor:
                cursor.execute(sql)
                datas = cursor.fetchall()
                return True, datas
        except Exception as e:
            return False, tuple()

    def close(self):
        self.conn.close()


if __name__ == '__main__':
    db = DbManagement()
    sql = "insert into workspace(ws_name, ws_addr, description, date) values ('1', 'ws3', 'sfs', "
    ret, id = db.insert_data(sql)
    print(ret)
    # ret, data = db.search_data("select id, ws_name from workspace")
    # print(data)