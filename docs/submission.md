# Project Submission

Submit one ZIP archive containing all results, Python scripts, and the completed project documentation.

## ZIP filename

Use the following naming convention:

`Team_[teamletter].zip`

For example, Team B must submit:

`Team_B.zip`

## Required contents

The files must be placed directly inside the ZIP archive.

For Team B, the archive must contain:

| File | Description |
|---|---|
| `B_Day_1.csv` | Day 1 results |
| `B_Day_2.csv` | Day 2 results |
| `B_Day_3.csv` | Day 3 results |
| `B_Day_1.py` | Day 1 Python script |
| `B_Day_2.py` | Day 2 Python script |
| `B_Day_3.py` | Day 3 Python script |
| `B_documentation.docx` | Completed project documentation |

!!! warning "Submission requirements"

    Upload exactly one ZIP archive per team. Replace `B` with your assigned team letter.  
    Do not rename the required files or place them inside an additional subfolder.

## Upload

<a href="https://laesze-my.sharepoint.com/:f:/g/personal/krecht_rudolf_sze_hu/IgB7AkvgJfBMRrlJ_UajW7IqAYlKPnQ3m-pMsXTY0HEooCA?e=xNYp9g"
   class="md-button md-button--primary"
   target="_blank"
   rel="noopener noreferrer">
  Upload project
</a>

## Random Team Order Generator

Enter the team letters or names separated by commas or new lines, then generate a random presentation order.

<div id="team-order-generator">
  <label for="team-list"><strong>Teams</strong></label>

  <textarea
    id="team-list"
    rows="4"
    placeholder="Team A, Team B, Team C, Team D"
    style="display:block; width:100%; max-width:600px; margin:0.5rem 0 1rem; padding:0.6rem;"
  ></textarea>

  <button
    id="generate-team-order"
    type="button"
    class="md-button md-button--primary">
    Generate random order
  </button>

  <p id="team-order-message" aria-live="polite"></p>
  <ol id="team-order-result"></ol>
</div>

<script>
(function () {
  const teamInput = document.getElementById("team-list");
  const generateButton = document.getElementById("generate-team-order");
  const message = document.getElementById("team-order-message");
  const result = document.getElementById("team-order-result");

  generateButton.addEventListener("click", function () {
    const teams = [
      ...new Set(
        teamInput.value
          .split(/[\n,;]+/)
          .map(team => team.trim())
          .filter(team => team.length > 0)
      )
    ];

    result.replaceChildren();

    if (teams.length < 2) {
      message.textContent = "Please enter at least two teams.";
      return;
    }

    for (let i = teams.length - 1; i > 0; i--) {
      const randomIndex = Math.floor(Math.random() * (i + 1));
      [teams[i], teams[randomIndex]] = [teams[randomIndex], teams[i]];
    }

    teams.forEach(function (team) {
      const listItem = document.createElement("li");
      listItem.textContent = team;
      result.appendChild(listItem);
    });

    message.textContent = "Random order generated:";
  });
})();
</script>