import string
import random
import time

class Functions:
    def get_random_digit():
        return random.randint(0,9)
    
    get_random_digit_JSON = {
        "name": "get_random_digit",
        "description": "Get a random digit",
        "parameters": {
            "type": "object",
            "properties": {},
        }
    }

    def get_random_letters(count: int, case_sensitive: bool = False):
        return ''.join(random.choices(string.ascii_letters if case_sensitive else string.ascii_uppercase, k=count))

    get_random_letters_JSON = {
        "name": "get_random_letters",
        "description": "Get a string of random letters",
        "parameters": {
            "type": "object",
            "properties": {
                "count": {"type": "integer", "description": "Number of letters to return"},
                "case_sensitive": {"type": "boolean", "description": "Whether to include lower-case letters.  Default only returns upper-case letters."}
            },
            "required": ["count"]
        }
    }

    def go_to_location(location):

        time.sleep(1)
        location="kitchen"

        return f"""i have reached the {location}.\n"""

    go_to_location_JSON= {
        "name": "go_to_location",
        "description": "go to location as requested by user",
        "parameters": {
            "type": "object",
            "properties": {
            "location": {
                "type": "string",
                "description": "name of the room like kitchen,bedroom,bathroom"
            }
            },
            "required": [
            "location"
            ]
        }
        }  
