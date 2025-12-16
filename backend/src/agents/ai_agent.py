"""
OpenAI agent service for the AI Agent with FastAPI application.
"""
import logging
import openai
from typing import Optional, Dict, Any, List
from src.config.settings import settings
from src.models.schemas import SourceReference

logger = logging.getLogger(__name__)


class OpenAIAgent:
    """
    Service class for interacting with OpenAI Assistant API
    """

    def __init__(self):
        """
        Initialize OpenAI client with configuration from settings
        """
        openai.api_key = settings.openai_api_key
        self.model = settings.openai_model
        self.instructions = settings.agent_instructions
        self.assistant_id = None

    async def create_assistant(self) -> str:
        """
        Create a new OpenAI assistant with custom instructions

        Returns:
            Assistant ID
        """
        try:
            assistant = openai.beta.assistants.create(
                name="Book Content Assistant",
                description="An AI assistant that answers questions based on retrieved book content",
                instructions=self.instructions,
                model=self.model,
            )
            self.assistant_id = assistant.id
            logger.info(f"Created new assistant with ID: {self.assistant_id}")
            return self.assistant_id
        except Exception as e:
            logger.error(f"Error creating assistant: {e}")
            raise

    async def get_assistant(self, assistant_id: str = None) -> Any:
        """
        Get an existing assistant or create a new one

        Args:
            assistant_id: Optional assistant ID to retrieve

        Returns:
            Assistant object
        """
        try:
            if assistant_id:
                assistant = openai.beta.assistants.retrieve(assistant_id)
                self.assistant_id = assistant_id
                return assistant
            elif self.assistant_id:
                assistant = openai.beta.assistants.retrieve(self.assistant_id)
                return assistant
            else:
                return await self.create_assistant()
        except Exception as e:
            logger.error(f"Error getting assistant: {e}")
            raise

    async def create_thread(self) -> str:
        """
        Create a new conversation thread

        Returns:
            Thread ID
        """
        try:
            thread = openai.beta.threads.create()
            logger.info(f"Created new thread with ID: {thread.id}")
            return thread.id
        except Exception as e:
            logger.error(f"Error creating thread: {e}")
            raise

    async def add_message_to_thread(self, thread_id: str, message: str) -> str:
        """
        Add a message to an existing thread

        Args:
            thread_id: Thread ID to add message to
            message: Message content

        Returns:
            Message ID
        """
        try:
            message_obj = openai.beta.threads.messages.create(
                thread_id=thread_id,
                role="user",
                content=message
            )
            logger.info(f"Added message to thread {thread_id}")
            return message_obj.id
        except Exception as e:
            logger.error(f"Error adding message to thread: {e}")
            raise

    async def run_assistant(self, thread_id: str, assistant_id: str = None) -> str:
        """
        Run the assistant on a thread and wait for completion

        Args:
            thread_id: Thread ID to run assistant on
            assistant_id: Optional assistant ID (uses default if not provided)

        Returns:
            Run ID
        """
        try:
            if not assistant_id:
                await self.get_assistant()
                assistant_id = self.assistant_id

            run = openai.beta.threads.runs.create(
                thread_id=thread_id,
                assistant_id=assistant_id
            )
            logger.info(f"Started assistant run with ID: {run.id}")
            return run.id
        except Exception as e:
            logger.error(f"Error running assistant: {e}")
            raise

    async def wait_for_run_completion(self, thread_id: str, run_id: str) -> Any:
        """
        Wait for a run to complete and return the result

        Args:
            thread_id: Thread ID
            run_id: Run ID to wait for

        Returns:
            Run object with completion status
        """
        try:
            import time
            while True:
                run = openai.beta.threads.runs.retrieve(
                    thread_id=thread_id,
                    run_id=run_id
                )
                if run.status in ["completed", "failed", "cancelled", "expired"]:
                    logger.info(f"Run {run_id} completed with status: {run.status}")
                    return run
                time.sleep(1)  # Wait 1 second before checking again
        except Exception as e:
            logger.error(f"Error waiting for run completion: {e}")
            raise

    async def get_thread_messages(self, thread_id: str) -> List[Dict[str, Any]]:
        """
        Get all messages from a thread

        Args:
            thread_id: Thread ID to get messages from

        Returns:
            List of message dictionaries
        """
        try:
            messages = openai.beta.threads.messages.list(
                thread_id=thread_id
            )
            # Convert to list of dictionaries
            message_list = []
            for msg in messages.data:
                message_dict = {
                    "id": msg.id,
                    "role": msg.role,
                    "content": msg.content[0].text.value if msg.content and len(msg.content) > 0 else "",
                    "timestamp": msg.created_at
                }
                message_list.append(message_dict)
            logger.info(f"Retrieved {len(message_list)} messages from thread {thread_id}")
            return message_list
        except Exception as e:
            logger.error(f"Error getting thread messages: {e}")
            raise

    async def process_query_with_context(
        self,
        query: str,
        thread_id: Optional[str] = None,
        context_sources: Optional[List[SourceReference]] = None
    ) -> str:
        """
        Process a query with additional context from retrieved sources

        Args:
            query: User query to process
            thread_id: Optional thread ID (creates new if not provided)
            context_sources: Optional list of sources to provide context

        Returns:
            Assistant response
        """
        try:
            # Create or use existing thread
            if not thread_id:
                thread_id = await self.create_thread()

            # Add context to the query if sources are provided
            full_query = query
            if context_sources:
                context_text = "\n\nAdditional context:\n"
                for i, source in enumerate(context_sources):
                    context_text += f"Source {i+1}: {source.section}\n"
                    context_text += f"Content: {source.text}\n"
                    context_text += f"URL: {source.url}\n\n"
                full_query = query + context_text

            # Add the query to the thread
            await self.add_message_to_thread(thread_id, full_query)

            # Run the assistant
            run_id = await self.run_assistant(thread_id)

            # Wait for completion
            run = await self.wait_for_run_completion(thread_id, run_id)

            if run.status == "completed":
                # Get the response messages
                messages = await self.get_thread_messages(thread_id)
                # Return the last assistant message
                for msg in reversed(messages):
                    if msg["role"] == "assistant":
                        return msg["content"]
                return "No response generated by assistant"
            else:
                raise Exception(f"Assistant run failed with status: {run.status}")

        except Exception as e:
            logger.error(f"Error processing query with context: {e}")
            raise


# Global instance of OpenAIAgent
openai_agent = OpenAIAgent()