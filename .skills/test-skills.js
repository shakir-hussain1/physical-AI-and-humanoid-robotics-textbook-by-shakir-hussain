/**
 * Test Suite for Claude Code Skills
 * Validates that all skills are properly structured and follow the expected format
 */

const fs = require('fs');
const path = require('path');

// Define the expected structure for skills
const EXPECTED_SECTIONS = [
  'Purpose',
  'Context',
  'Guidelines',
  'Input Format',
  'Output Format'
];

// Define the skills directory structure
const SKILLS_DIR = './.skills';
const SKILL_CATEGORIES = [
  'content-generation',
  'quality-assurance',
  'structure-organization',
  'interactive-elements'
];

function validateSkillStructure(skillPath) {
  try {
    const content = fs.readFileSync(skillPath, 'utf8');
    const lines = content.split('\n');

    // Check for required sections - look for Markdown headers (## Section)
    const foundSections = [];
    lines.forEach(line => {
      // Check for Markdown headers (## Section)
      if (line.trim().startsWith('## ')) {
        const sectionTitle = line.replace('## ', '').trim();
        // Only add if it's one of our expected sections
        if (EXPECTED_SECTIONS.includes(sectionTitle)) {
          foundSections.push(sectionTitle);
        }
      }
    });

    // Validate required sections are present
    const missingSections = EXPECTED_SECTIONS.filter(
      section => !foundSections.includes(section)
    );

    return {
      isValid: missingSections.length === 0,
      missingSections,
      foundSections,
      path: skillPath
    };
  } catch (error) {
    return {
      isValid: false,
      error: error.message,
      path: skillPath
    };
  }
}

function testSkills() {
  console.log('🧪 Testing Claude Code Skills Structure...\n');

  let totalSkills = 0;
  let validSkills = 0;
  let issuesFound = [];

  SKILL_CATEGORIES.forEach(category => {
    const categoryPath = path.join(SKILLS_DIR, category);

    if (!fs.existsSync(categoryPath)) {
      console.log(`⚠️  Category directory not found: ${categoryPath}`);
      return;
    }

    const skills = fs.readdirSync(categoryPath).filter(file =>
      file.endsWith('.skill')
    );

    console.log(`📋 Testing ${category} (${skills.length} skills):`);

    skills.forEach(skill => {
      totalSkills++;
      const skillPath = path.join(categoryPath, skill);
      const result = validateSkillStructure(skillPath);

      if (result.isValid) {
        validSkills++;
        console.log(`  ✅ ${skill}`);
      } else {
        console.log(`  ❌ ${skill} - Issues: ${result.missingSections ? result.missingSections.join(', ') : result.error}`);
        issuesFound.push(result);
      }
    });

    console.log(''); // Empty line after each category
  });

  // Summary
  console.log('📊 Summary:');
  console.log(`Total Skills: ${totalSkills}`);
  console.log(`Valid Skills: ${validSkills}`);
  console.log(`Invalid Skills: ${totalSkills - validSkills}`);
  console.log(`Success Rate: ${totalSkills > 0 ? Math.round((validSkills / totalSkills) * 100) : 0}%`);

  if (issuesFound.length > 0) {
    console.log('\n❌ Issues Found:');
    issuesFound.forEach((issue, index) => {
      console.log(`${index + 1}. ${issue.path}`);
      if (issue.missingSections) {
        console.log(`   Missing sections: ${issue.missingSections.join(', ')}`);
      }
      if (issue.error) {
        console.log(`   Error: ${issue.error}`);
      }
    });
  } else {
    console.log('\n🎉 All skills are properly structured!');
  }

  // Additional validation: Check config.json
  console.log('\n📋 Validating configuration...');
  const configPath = path.join(SKILLS_DIR, 'config.json');
  if (fs.existsSync(configPath)) {
    try {
      const config = JSON.parse(fs.readFileSync(configPath, 'utf8'));
      console.log('✅ Configuration file is valid JSON');
      console.log(`📚 Project: ${config.name}`);
      console.log(`🔢 Version: ${config.version}`);
    } catch (error) {
      console.log(`❌ Configuration file error: ${error.message}`);
    }
  } else {
    console.log('❌ Configuration file not found');
  }

  return {
    total: totalSkills,
    valid: validSkills,
    invalid: totalSkills - validSkills,
    successRate: totalSkills > 0 ? Math.round((validSkills / totalSkills) * 100) : 0,
    issues: issuesFound
  };
}

// Run the test
if (require.main === module) {
  testSkills();
}

module.exports = { testSkills, validateSkillStructure, EXPECTED_SECTIONS };