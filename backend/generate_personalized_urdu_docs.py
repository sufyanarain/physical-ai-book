import os
import sys
import asyncio
from groq import AsyncGroq
from pathlib import Path
from dotenv import load_dotenv

# Fix Windows console encoding for emojis
if sys.platform == "win32":
    sys.stdout.reconfigure(encoding='utf-8')

# Load environment variables
load_dotenv()

async def personalize_urdu_content(client, content, background_type):
    """Use Groq LLM to personalize Urdu content for specific background."""
    
    if background_type == "software":
        system_prompt = """
        You are adapting Urdu technical documentation for SOFTWARE DEVELOPERS.
        
        IMPORTANT:
        1. Keep ALL Urdu text in Urdu
        2. Keep frontmatter unchanged
        3. Keep code blocks in English
        4. Keep markdown formatting exactly
        
        ADAPT THE CONTENT BY:
        - Adding detailed explanations of hardware/electronics concepts in Urdu
        - Using programming analogies (APIs, frameworks, operating systems)
        - Assume knowledge of: Python, OOP, algorithms, data structures
        - Explain in Urdu: Circuits, sensors, actuators, motors, mechanical systems
        - Add software developer perspective to Urdu explanations
        """
    else:  # hardware
        system_prompt = """
        You are adapting Urdu technical documentation for HARDWARE/ELECTRONICS ENGINEERS.
        
        IMPORTANT:
        1. Keep ALL Urdu text in Urdu
        2. Keep frontmatter unchanged
        3. Keep code blocks in English
        4. Keep markdown formatting exactly
        
        ADAPT THE CONTENT BY:
        - Adding detailed explanations of programming/software concepts in Urdu
        - Using electronics analogies (microcontrollers, circuits, sensors)
        - Assume knowledge of: Electronics, mechanics, circuits, sensors
        - Explain in Urdu: Python syntax, OOP, algorithms, software design patterns
        - Add hardware engineer perspective to Urdu explanations
        """
    
    try:
        response = await client.chat.completions.create(
            model="llama-3.3-70b-versatile",
            messages=[{
                "role": "system",
                "content": system_prompt
            }, {
                "role": "user",
                "content": f"Adapt this Urdu documentation:\n\n{content}"
            }],
            temperature=0.3,
            max_tokens=8000
        )
        return response.choices[0].message.content
    except Exception as e:
        print(f"Error personalizing content: {e}")
        return content  # Return original on error

async def generate_personalized_urdu_docs():
    """Generate personalized Urdu documentation for both backgrounds."""
    client = AsyncGroq(api_key=os.getenv("GROQ_API_KEY"))

    # Define paths
    source_dir = Path("../website/docs-urdu")
    software_dir = Path("../website/docs-urdu-software")
    hardware_dir = Path("../website/docs-urdu-hardware")

    print("🚀 Starting personalized Urdu documentation generation...")
    print(f"Source: {source_dir.absolute()}")
    print(f"Software version: {software_dir.absolute()}")
    print(f"Hardware version: {hardware_dir.absolute()}\n")

    # Create directories
    software_dir.mkdir(exist_ok=True)
    hardware_dir.mkdir(exist_ok=True)

    # Find all markdown files
    md_files = list(source_dir.rglob("*.md"))
    total = len(md_files)
    print(f"📚 Found {total} Urdu markdown files to personalize\n")

    for idx, md_file in enumerate(md_files, 1):
        try:
            content = md_file.read_text(encoding='utf-8')
            relative_path = md_file.relative_to(source_dir)

            print(f"[{idx}/{total}] Processing: {relative_path}")

            # Generate software background version
            print(f"  → Adapting for software developers...")
            software_content = await personalize_urdu_content(client, content, "software")
            
            # Add delay to avoid rate limits
            await asyncio.sleep(3)

            # Generate hardware background version
            print(f"  → Adapting for hardware engineers...")
            hardware_content = await personalize_urdu_content(client, content, "hardware")
            
            # Add delay to avoid rate limits
            await asyncio.sleep(3)

            # Save files maintaining directory structure
            software_file = software_dir / relative_path
            software_file.parent.mkdir(parents=True, exist_ok=True)
            software_file.write_text(software_content, encoding='utf-8')

            hardware_file = hardware_dir / relative_path
            hardware_file.parent.mkdir(parents=True, exist_ok=True)
            hardware_file.write_text(hardware_content, encoding='utf-8')

            print(f"  ✓ Both versions complete\n")

        except Exception as e:
            print(f"  ✗ Error processing {relative_path}: {e}\n")
            # Wait longer on error (likely rate limit)
            await asyncio.sleep(10)
            continue

    print("\n✅ Personalized Urdu documentation generation complete!")
    print(f"📁 Software version: {software_dir.absolute()}")
    print(f"📁 Hardware version: {hardware_dir.absolute()}")

if __name__ == "__main__":
    asyncio.run(generate_personalized_urdu_docs())
