from __future__ import print_function
import time
import sib_api_v3_sdk
from sib_api_v3_sdk.rest import ApiException
from pprint import pprint


## This function will send email to target email address, which is convenient if you want to get notified when bug happens or robot stops.

def send_email_with_error_info():
    configuration = sib_api_v3_sdk.Configuration()
    configuration.api_key['api-key'] = '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'      ## Xili's debugging only email API, you can use it if you want

    api_instance = sib_api_v3_sdk.TransactionalEmailsApi(sib_api_v3_sdk.ApiClient(configuration))
    subject = "Error warning"
    html_content = "<html><body><h1>ERROR HAPPENED</h1></body></html>"
    sender = {"name":"Medusa MMint","email":"eclymktest@gmail.com"}                         ## Xili's debugging only email API, you can use it if you want
    to = [{"email":"eclymk@gmail.com","name":"Xili"}]

    headers = {"Some-Custom-Name":"unique-id-1234"}
    params = {"parameter":"My param value","subject":"New Subject"}
    send_smtp_email = sib_api_v3_sdk.SendSmtpEmail(to=to, headers=headers, html_content=html_content, sender=sender, subject=subject)

    try:
        api_response = api_instance.send_transac_email(send_smtp_email)
        pprint(api_response)
    except ApiException as e:
        print("Exception when calling SMTPApi->send_transac_email: %s\n" % e)

