class Projectable(object):
    def __init__(self, service_proxy, service_request):
        self.service_proxy   = service_proxy
        self.service_request = service_request
        
    def itemClicked(self):
        pass