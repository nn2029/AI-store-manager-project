import xmlrpc.client
import pandas as pd

# Odoo details
url = 'http://your-odoo-server.com'
db = 'your-db-name'
username = 'your-username'
password = 'your-password'

def update_odoo_from_csv():
    df = pd.read_csv('$(find store_management_robot)/csv/product_status.csv')
    
    common = xmlrpc.client.ServerProxy(f'{url}/xmlrpc/2/common')
    uid = common.authenticate(db, username, password, {})
    models = xmlrpc.client.ServerProxy(f'{url}/xmlrpc/2/object')
    
    for index, row in df.iterrows():
        product_name = row['product_id']
        count = row['product_count']
        status = row['status']



        models.execute_kw(db, uid, password, 'product.product', 'write', [[product_name], {'count': count}])
        
if __name__ == '__main__':
    update_odoo_from_csv()

