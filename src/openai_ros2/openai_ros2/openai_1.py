import json
from openai import OpenAI
import time
import os


client = OpenAI(
    api_key=os.getenv("OPEN_AI_KEY")
)
  

assistant_id="asst_KTFlL51Og2srFF5bqmaaKM9k"


def go_to_location(location):

    time.sleep(1)
    location="kitchen"

    return f"""i have reached the {location}.\n"""


assistant=client.beta.assistants.retrieve(
    assistant_id=assistant_id
)

thread=client.beta.threads.create()

run=client.beta.threads.runs.create(
    thread_id=thread.id,
    assistant_id=assistant.id,
    instructions="go to the kitchen"
)

def get_outputs_for_tool_call(tool_call):
    location=json.loads(tool_call.function.arguments)["location"]
    location_details=go_to_location(location=location)
    return{
        "tool_call_id":tool_call.id,
        "output":location_details
    }

tools_calls=run.required_action.submit_tool_outputs.tool_calls
tool_outputs=map(get_outputs_for_tool_call,tools_calls)
tool_outputs=list(tool_outputs)

tool_outputs

run=client.beta.threads.runs.submit_tool_outputs(
    thread_id=thread.id,
    run_id=run.id,
    tool_outputs=tool_outputs)
