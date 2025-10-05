import yaml

class RobotConfigManager:
    def __init__(self, config_file='/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_rokey/pick_and_place_voice/robot_control/data/set_config.yaml'):
        """Initialize with config file path and load configuration."""
        self.config = self._load_config(config_file)
        if self.config:
            self._set_robot_attributes()
            self._set_llm_attributes()
            self._set_cigarette_map()

    def _load_config(self, file_path):
        """Load YAML configuration file."""
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                return yaml.safe_load(file)
        except Exception as e:
            print(f"Error loading YAML file: {e}")
            return None

    def _set_robot_attributes(self):
        """Set robot-related attributes from config."""
        robot_data = self.config.get('robot', {})
        self.robot_id = robot_data.get('ROBOT_ID')
        self.robot_model = robot_data.get('ROBOT_MODEL')
        self.velocity = robot_data.get('VELOCITY')
        self.acceleration = robot_data.get('ACC')
        self.gripper_name = robot_data.get('GRIPPER_NAME')
        self.toolcharger_ip = robot_data.get('TOOLCHARGER_IP')
        self.toolcharger_port = robot_data.get('TOOLCHARGER_PORT')
        self.depth_offset = robot_data.get('DEPTH_OFFSET')
        self.min_depth = robot_data.get('MIN_DEPTH')

    def _set_llm_attributes(self):
        """Set LLM-related attributes from config."""
        llm_data = self.config.get('llm', {})
        self.openai_api_key = llm_data.get('OPENAI_API_KEY')
        self.max_retries = llm_data.get('MAX_RETRIES')

    def _set_cigarette_map(self):
        """Set cigarette name map from config."""
        self.cigarette_name_map = self.config.get('CIGARETTE_NAME_MAP', {})

    def generate_confirmation_prompt(self, brands, counts):
        """Generate order confirmation prompt."""
        prompt_parts = []
        for brand, count in zip(brands, counts):
            korean_name = self.cigarette_name_map.get(brand, brand)
            unit = "보루" if int(count) == 10 else "갑"
            prompt_parts.append(f"{korean_name} {count}{unit}")
        return ", ".join(prompt_parts) + " 맞으신가요?"

# Example usage
if __name__ == "__main__":
    # Initialize config manager
    config_manager = RobotConfigManager()

    # Access robot attributes
    print("Robot Information:")
    print(f"Robot ID: {config_manager.robot_id}")
    print(f"Robot Model: {config_manager.robot_model}")
    print(f"Velocity: {config_manager.velocity}")
    print(f"Acceleration: {config_manager.acceleration}")
    print(f"Gripper: {config_manager.gripper_name}")
    print(f"Toolcharger IP: {config_manager.toolcharger_ip}")
    print(f"Toolcharger Port: {config_manager.toolcharger_port}")
    print(f"Depth Offset: {config_manager.depth_offset}")
    print(f"Min Depth: {config_manager.min_depth}")

    # Access LLM attributes
    print("\nLLM Configuration:")
    print(f"OpenAI API Key: {config_manager.openai_api_key}")
    print(f"Max Retries: {config_manager.max_retries}")

    # Example: Generate confirmation prompt
    print("\nOrder Confirmation Example:")
    brands = ['esse', 'mevius', 'unknown_brand']
    counts = ['1', '10', '2']
    prompt = config_manager.generate_confirmation_prompt(brands, counts)
    print(prompt)