import re
with open('c:/Users/jayan.kandathil/Documents/andrew-robot_test/node_ui/index.html', 'r', encoding='utf-8') as f:
    text = f.read()

# Replace the closing div before Right Column and open tab-map
old = '''      </div>

      <!-- Right Column: Workspace Map -->'''

new = '''      </div>
    </div> <!-- Close tab-dashboard -->

    <!-- Right Column: Workspace Map -->
    <div id="tab-map" style="display: none; align-items: center; justify-content: center; flex-direction: column;">'''

text = text.replace(old, new)

# Close tab map before the script
text = text.replace('    </div>\n  </div>\n\n  <script>', '      </div>\n    </div>\n  </div> <!-- Close tab-map -->\n\n  <script>')

with open('c:/Users/jayan.kandathil/Documents/andrew-robot_test/node_ui/index.html', 'w', encoding='utf-8') as f:
    f.write(text)
