import { createConsumer } from "@rails/actioncable"

// Function to get the meta tag content
const getMetaValue = (name) => {
  const element = document.head.querySelector(`meta[name="${name}"]`)
  return element ? element.getAttribute("content") : null
}

// Function to create a consumer URL with potential token
const createURL = () => {
  const cableUrl = getMetaValue("action-cable-url") // Get base URL from meta tag
  const userIdToken = getMetaValue("current-user-id") // Get signed user ID token
  
  // Log the values read from meta tags
  console.log(`[Consumer] Read action-cable-url: ${cableUrl}`);
  console.log(`[Consumer] Read current-user-id token: ${userIdToken}`);

  if (userIdToken && cableUrl) {
    // Append token as a query parameter
    const url = new URL(cableUrl)
    url.searchParams.append("user_token", userIdToken)
    console.log("Creating Action Cable consumer with URL:", url.toString()) // Log the URL
    return url.toString()
  } else if (cableUrl) {
    // Fix newline in console log
    console.log("Creating Action Cable consumer with default URL (no user token found): " + cableUrl)
    return cableUrl
  } else {
    // Fallback if meta tag isn't present (should ideally not happen)
    console.error("Action Cable URL meta tag not found. Falling back to /cable")
    return "/cable"
  }
}

export default createConsumer(createURL()) 