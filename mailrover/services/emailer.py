import smtplib
import mimetypes
import os
from email.message import EmailMessage


class Emailer:
    def __init__(self, config, logger):
        self.config = config
        self.logger = logger

    def send(self, to_address: str, subject: str, text_body: str, html_body: str = None) -> dict:
        to_address = (to_address or "").strip()
        if not to_address:
            return {"ok": False, "mode": "none", "error": "MISSING_RECIPIENT_EMAIL"}

        if not getattr(self.config, "EMAIL_ENABLED", False) or not getattr(self.config, "SMTP_HOST", ""):
            self.logger.log(
                "EMAIL_DEV_LOG",
                f"to={to_address} subject={subject} body={text_body.replace(chr(10), ' / ')}"
            )
            return {"ok": True, "mode": "logged"}

        msg = EmailMessage()
        msg["From"] = getattr(self.config, "EMAIL_FROM", "")
        msg["To"] = to_address
        msg["Subject"] = subject
        msg.set_content(text_body)

        if html_body:
            msg.add_alternative(html_body, subtype="html")
            self._attach_inline_logo(msg)

        try:
            with smtplib.SMTP(
                getattr(self.config, "SMTP_HOST"),
                getattr(self.config, "SMTP_PORT", 587),
                timeout=5,
            ) as smtp:
                if getattr(self.config, "SMTP_USE_TLS", True):
                    smtp.starttls()

                username = getattr(self.config, "SMTP_USERNAME", "")
                password = getattr(self.config, "SMTP_PASSWORD", "")
                if username and password:
                    smtp.login(username, password)

                smtp.send_message(msg)

            self.logger.log("EMAIL_SENT", f"to={to_address} subject={subject}")
            return {"ok": True, "mode": "smtp"}
        except Exception as e:
            self.logger.log("EMAIL_SEND_FAIL", f"to={to_address} subject={subject} err={e}")
            return {"ok": False, "mode": "smtp", "error": "SMTP_SEND_FAILED"}

    def _attach_inline_logo(self, msg: EmailMessage):
        logo_path = getattr(self.config, "EMAIL_LOGO_PATH", "")
        if not logo_path or not os.path.exists(logo_path):
            return

        html_part = msg.get_payload()[-1]
        mime_type, _ = mimetypes.guess_type(logo_path)
        maintype, subtype = (mime_type or "image/png").split("/", 1)

        with open(logo_path, "rb") as logo_file:
            html_part.add_related(
                logo_file.read(),
                maintype=maintype,
                subtype=subtype,
                cid="<mailrover-logo>",
                filename=os.path.basename(logo_path),
            )
